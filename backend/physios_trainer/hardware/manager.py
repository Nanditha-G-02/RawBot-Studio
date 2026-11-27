# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Flexible robot setup: arm discovery, connection state, monitoring, recovery, calibration and
supervised jog. Used by the training service (server.py routes under /hardware/).

Ownership of the serial ports
    bridge.py (the operator's ROS driver) owns the ports while it runs. RawBot Studio opens a
    port only when no other program holds it: for a motor scan, a read-only monitor session
    (Connect) or a calibration. Everything that moves the arm (teleoperation, recording,
    rollout, jog) goes through bridge.py over ROS.

When is an arm "connected"
    Only after communication is verified: every motor answered in RawBot Studio's own
    read-only session, or bridge.py publishes live joint messages for it on ROS. A port that
    merely exists, or a process that merely holds it, is never reported as connected.
"""

from collections import deque
import json
import math
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid

from physios_trainer.hardware import capabilities as capmod
from physios_trainer.hardware import kinematics as kinmod
from physios_trainer.hardware import planning as planmod
from physios_trainer.hardware import ports as portmod
from physios_trainer.hardware import robotcaps
from physios_trainer.hardware.registry import MODES, ROLES, Registry, motor_spec
from physios_trainer.hardware.relay import StateRelay
from physios_trainer.hardware.simbot import SIM_TOPIC, SimRobot
from physios_trainer.util import atomic_write_json, now_iso, read_json

HERE = Path(__file__).resolve().parent
WORKER = HERE / 'hw_worker.py'
JOG_WORKER = HERE / 'jog_worker.py'
ROS_LIVE_S = 1.0          # joint messages younger than this = live
PROBE_TTL_S = 300         # a motor scan validates an adapter for this long (role assignment)


class HardwareError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def _alive(pid):
    if not pid:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    try:
        with open('/proc/%d/stat' % pid) as f:
            return f.read().split()[2] != 'Z'
    except OSError:
        return True


# ------------------------------------------------------------------ ROS watch ---
class RosWatch:
    """Keeps one rosbridge connection and the latest joint messages of the watched topics.
    For the leader topic it also asks rosapi who publishes, so commands RawBot Studio sends
    itself (jog, rollout) are never mistaken for a live leader arm."""

    def __init__(self, url, log=print):
        self.url = url
        self.log = log
        self.topics = {}            # topic -> type
        self.pub_topics = set()     # topics whose publishers are checked (not subscribed)
        self.publishers = {}        # topic -> (t, [names]) from rosapi
        self.br = None
        self.connected = False
        self.error = None
        self.lock = threading.RLock()
        self._subscribed = set()
        self._types = {}            # topic -> type it is subscribed with
        self._stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def watch(self, topics, publishers_of=(), frames=()):
        """frames: camera topics whose frame age is watched (1 frame/s is enough)."""
        with self.lock:
            self.topics = dict(topics)
            self.pub_topics = {t for t in publishers_of if t}
            self.frame_topics = {t for t in frames if t}

    def _loop(self):
        from physios_trainer.rollout_worker import Bridge, drop
        last_pub = 0.0
        while not self._stop:
            with self.lock:
                topics = dict(self.topics)
                pub_only = set(self.pub_topics) - set(topics)
                frames = set(getattr(self, 'frame_topics', ())) - set(topics)
            if not topics and not pub_only and not frames:
                time.sleep(0.5)
                continue
            if self.br is None or not self.br.alive:
                self.br = drop(self.br)
                try:
                    self.br = Bridge(self.url)
                    self._subscribed = set()
                    self._types = {}
                    self.connected = True
                    self.error = None
                except Exception as e:  # noqa: BLE001
                    self.connected = False
                    self.error = str(e)[:200]
                    self.br = None
                    time.sleep(2.0)
                    continue
            try:
                for t, typ in topics.items():
                    if t not in self._subscribed:
                        typ = self._real_type(t) or typ
                        self.br.subscribe(t, typ, throttle_ms=100)
                        self._subscribed.add(t)
                        self._types[t] = typ
                for t in frames:
                    if t not in self._subscribed:
                        self.br.subscribe(t, 'sensor_msgs/msg/CompressedImage', throttle_ms=1000)
                        self._subscribed.add(t)
                if time.time() - last_pub > 2.5:
                    last_pub = time.time()
                    # a role's usual type is a guess (a leader is a JointTrajectory from
                    # bridge.py, but an arm's own driver publishes JointState): a topic that
                    # stays silent is re-subscribed with the type it really has
                    for t in list(topics):
                        if t in self.br.latest:
                            continue
                        real = self._real_type(t)
                        if real and real != self._types.get(t):
                            self.br.unsubscribe(t)
                            self.br.subscribe(t, real, throttle_ms=100)
                            self._types[t] = real
                    for t, typ in list(topics.items()) + [(t, '') for t in pub_only]:
                        if 'JointTrajectory' in typ or t in self.pub_topics:
                            try:
                                pubs = self.br.call('/rosapi/publishers', {'topic': t},
                                                    timeout=1.5).get('publishers') or []
                                self.publishers[t] = (time.time(), list(pubs))
                            except (TimeoutError, RuntimeError):
                                self.publishers.pop(t, None)
            except Exception as e:  # noqa: BLE001
                self.error = str(e)[:200]
                self.connected = False
                self.br = drop(self.br)
            time.sleep(0.25)

    def _real_type(self, topic):
        """The topic's type from rosapi, or '' (no rosapi, topic not there yet)."""
        try:
            return (self.br.call('/rosapi/topic_type', {'topic': topic}, timeout=1.0)
                    .get('type') or '')
        except Exception:  # noqa: BLE001 - rosapi missing or slow: keep the usual type
            return ''

    def kind(self, topic):
        """'state' (sensor_msgs/JointState: an arm's own joints), 'command'
        (JointTrajectory: bridge.py's leader commands) or None (no message yet)."""
        br = self.br
        hit = br.latest.get(topic) if br else None
        if not hit:
            return None
        return 'command' if 'joint_names' in hit[1] else 'state'

    def sample(self, topic):
        """(age_s, names, positions) or (None, [], [])."""
        br = self.br
        hit = br.latest.get(topic) if br else None
        if not hit:
            return None, [], []
        t, msg = hit
        if 'joint_names' in msg:
            pts = msg.get('points') or [{}]
            return time.time() - t, list(msg.get('joint_names') or []), list(
                (pts[0] or {}).get('positions') or [])
        return time.time() - t, list(msg.get('name') or []), list(msg.get('position') or [])

    def age(self, topic):
        """Seconds since the last message on a watched topic, or None."""
        br = self.br
        hit = br.latest.get(topic) if br else None
        return time.time() - hit[0] if hit else None

    def foreign_publishers(self, topic):
        """None when unknown (rosapi not reachable), else publishers other than rosbridge."""
        hit = self.publishers.get(topic)
        if not hit or time.time() - hit[0] > 8:
            return None
        return [p for p in hit[1] if 'rosbridge' not in p]


# ------------------------------------------------------------------- manager ---
class Hardware:

    def __init__(self, root, envs, rollouts=None, notifier=None, log=print,
                 rosbridge_url=None, monitor=True, robots_dir=None):
        self.root = Path(root)
        self.reg = Registry(self.root)
        self.sess_dir = self.root / 'sessions'
        self.sess_dir.mkdir(parents=True, exist_ok=True)
        self.envs = envs
        self.rollouts = rollouts
        self.notifier = notifier
        self.log = log
        self.lock = threading.RLock()
        self.ros = RosWatch(rosbridge_url or os.environ.get('RAWBOT_ROSBRIDGE_URL',
                                                            'ws://127.0.0.1:9090'), log=log)
        self.monitors = {}           # (robot_type, role) -> {proc, dir, port, key}
        self.calib = None            # {id, proc, dir, role, robot_type, ...}
        self.jog = None
        self.probes = {}             # identity key -> probe result
        self.ports = []
        self.port_holders = {}
        self.ports_at = 0.0
        self.events = deque(maxlen=200)
        self.event_seq = 0
        self.prev = {}               # (robot_type, role) -> state, for transition events
        self.ever_connected = {}     # (robot_type, role) -> True once verified this session
        self.active_robot = None
        self.action_source_supported = {}
        self._jog_lock = threading.Lock()
        self.robots_dir = robots_dir     # custom robots (URDF joints and limits)
        # the simulated robot (simbot.py); publishes over its own rosbridge connection
        self.sim = SimRobot(self.ros.url, sampler=lambda t: self.ros.sample(t),
                            bridge_factory=_bridge if monitor else None, log=log,
                            thread=monitor)
        # the follower's joint states from RawBot Studio's serial session, when no bridge.py
        self.relay = StateRelay(self.ros.url, bridge_factory=_bridge if monitor else None,
                                log=log, thread=monitor)
        self._py_cache = {}
        self._ports_seen = False
        self._tick_ports = set()
        self._stop = False
        self._cleanup_stale_sessions()
        if monitor:
            threading.Thread(target=self._monitor_loop, daemon=True).start()

    # ---- helpers ------------------------------------------------------------------
    def _cleanup_stale_sessions(self):
        for d in self.sess_dir.glob('*'):
            st = read_json(d / 'status.json') or {}
            if st.get('pid') and _alive(st['pid']):
                # left over from a service restart: stop it, it has no owner any more
                atomic_write_json(d / 'control.json', {'command': 'stop', 't': time.time()})

    def _event(self, robot_type, kind, text, tone='info', notify=True):
        with self.lock:
            self.event_seq += 1
            ev = {'id': self.event_seq, 't': now_iso(), 'robot_type': robot_type,
                  'kind': kind, 'text': text, 'tone': tone}
            self.events.append(ev)
        self.log('hardware: %s' % text)
        if notify and self.notifier:
            try:
                self.notifier.notify('Robot hardware', text,
                                     {'ok': 'ok', 'warn': 'info', 'err': 'error'}.get(
                                         tone, 'info'), None)
            except Exception:  # noqa: BLE001
                pass
        return ev

    def _python(self, robot_type):
        """Interpreter for hw_worker: the operator's LeRobot environment."""
        override = os.environ.get('RAWBOT_HW_PYTHON')
        if override:
            return override, dict(os.environ)
        hit = self._py_cache.get(robot_type)
        if hit and time.time() - hit[0] < 60:
            return hit[1], hit[2]
        setup = self.reg.setup(robot_type)
        names = [setup['env']] if setup.get('env') else []
        envs = self.envs.list()
        names += ['lerobot'] + [e['name'] for e in envs]
        seen = set()
        for n in names:
            if not n or n in seen:
                continue
            seen.add(n)
            try:
                p = self.envs.probe(n)
            except KeyError:
                continue
            if p.get('ok') and (p.get('packages') or {}).get('lerobot'):
                env = self.envs.get(n)
                py, ev = os.path.join(env['prefix'], 'bin', 'python'), self.envs.env_vars(env)
                self._py_cache[robot_type] = (time.time(), py, ev)
                return py, ev
        raise HardwareError('No Python environment with LeRobot was found. Motor scans and '
                            'calibration use the LeRobot that drives your arm (conda env '
                            '"lerobot"). Training → Environment lists what was found.')

    def _own_pids(self):
        pids = set()
        for m in self.monitors.values():
            if m['proc'].poll() is None:
                pids.add(m['proc'].pid)
        if self.calib and self.calib['proc'].poll() is None:
            pids.add(self.calib['proc'].pid)
        return pids

    def refresh_ports(self, force=False):
        with self.lock:
            if not force and time.time() - self.ports_at < 0.3:
                return self.ports
            if portmod.supported():
                ps = portmod.list_ports()
            else:
                ps = self._worker_ports()
            hold = portmod.holders([p['device'] for p in ps], skip_pids=self._own_pids())
            own = {m['port']: m for m in self.monitors.values() if m['proc'].poll() is None}
            if self.calib and self.calib['proc'].poll() is None:
                own[self.calib['port']] = self.calib
            for p in ps:
                p['holders'] = hold.get(p['device'], [])
                p['own_session'] = 'calibration' if (self.calib and own.get(
                    p['device']) is self.calib) else ('monitor' if p['device'] in own else None)
            self.ports, self.ports_at = ps, time.time()
            return ps

    def _worker_ports(self):
        try:
            py, env = self._python(self.active_robot)
        except HardwareError:
            return []
        try:
            p = subprocess.run([py, '-I', str(WORKER), 'ports'], capture_output=True, text=True,
                               timeout=20, env=env)
        except (OSError, subprocess.SubprocessError):
            return []
        line = next((x for x in p.stdout.splitlines() if x.startswith('RAWBOT_HW ')), None)
        if not line:
            return []
        out = json.loads(line[10:]).get('ports') or []
        for x in out:
            x['key'] = portmod.identity_key(x)
            x['unique'] = bool(x.get('serial'))
            x['access'] = 'ok'
        return out

    def _port_for(self, dev):
        """Current port of a registered device: by identity, then by the operator's override."""
        ps = self.refresh_ports()
        for p in ps:
            if p['key'] == dev['key']:
                return p
        ov = dev.get('port_override')
        if ov:
            for p in ps:
                if p['device'] == ov or p.get('by_id') == ov:
                    return dict(p, override=True)
        return None

    def _probe(self, robot_type, port):
        spec = motor_spec(robot_type)
        if not spec:
            raise HardwareError('Motor scans are available for SO-101 arms (Feetech). This '
                                'robot type is detected through its ROS topics only.')
        if port.get('holders'):
            h = port['holders'][0]
            raise HardwareError('%s is in use by %s (PID %d). Stop it to scan the motors.' % (
                port['device'], h['name'], h['pid']), 409)
        if port.get('own_session'):
            raise HardwareError('%s is open in a RawBot Studio %s session.' % (
                port['device'], port['own_session']), 409)
        if port.get('access') == 'denied':
            raise HardwareError('No permission to open %s. Add your user to the "dialout" '
                                'group (sudo usermod -aG dialout $USER), then log in again.'
                                % port['device'], 403)
        py, env = self._python(robot_type)
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
            json.dump({'port': port['device'], 'motors': spec}, f)
            cfg = f.name
        try:
            p = subprocess.run([py, '-I', str(WORKER), 'probe', cfg], capture_output=True,
                               text=True, timeout=30, env=env)
        except subprocess.TimeoutExpired:
            raise HardwareError('The motor scan on %s did not finish in 30 s.' % port['device'])
        finally:
            os.unlink(cfg)
        line = next((x for x in p.stdout.splitlines() if x.startswith('RAWBOT_HW ')), None)
        if not line:
            raise HardwareError(explain_serial_error((p.stderr or p.stdout)[-400:],
                                                     port['device']))
        res = json.loads(line[10:])
        if res.get('error'):
            res['error'] = explain_serial_error(res['error'], port['device'])
        res['device'] = port['device']
        res['checked_at'] = time.time()
        self.probes[port['key']] = res
        return res

    # ---- setup ------------------------------------------------------------------------
    def setup(self, robot_type, mode=None, topics=None, env=None, source='operator',
              cameras=None, joints=None, sim_twin=None, selection=None, joint_map=None,
              joint_pairs=None, workspace=None, workspace_guard=None, sim_cameras=None):
        cur = self.reg.setup(robot_type)
        kw = {}
        if sim_cameras is not None:
            kw['sim_cameras'] = bool(sim_cameras)
        if isinstance(workspace, dict):
            num = {k: float(v) for k, v in workspace.items() if isinstance(v, (int, float))
                   and k.endswith('_mm')}
            num['keepouts'] = [k for k in workspace.get('keepouts') or []
                               if isinstance(k, dict)][:20]
            kw['workspace'] = num
        if workspace_guard is not None:
            kw['workspace_guard'] = bool(workspace_guard)
        if joint_map is not None:
            kw['joint_map'] = clean_joint_map(joint_map)
        if joint_pairs is not None:
            kw['joint_pairs'] = {str(k)[:60]: str(v)[:60] for k, v in (joint_pairs or {}).items()
                                 if isinstance(v, str) and v and v != k}
        if cameras is not None:
            kw['cameras'] = {str(k)[:40]: t for k, t in (cameras or {}).items()
                             if isinstance(t, str) and t.startswith('/') and len(t) < 200}
        if joints is not None:
            kw['joints'] = [str(n)[:60] for n in (joints or [])][:32]
        if sim_twin is not None:
            kw['sim_twin'] = bool(sim_twin)
        if selection is not None:
            sel = {k: selection.get(k) for k in ('action', 'target', 'observation')
                   if isinstance(selection.get(k), str)}
            kw['selection'] = dict(cur.get('selection') or {}, **sel)
        if mode is not None:
            if mode not in MODES:
                raise HardwareError('Unknown hardware configuration "%s".' % mode)
            kw['mode'] = mode
            kw['mode_source'] = source
            if mode != cur['mode'] and selection is None:
                # a new configuration starts from its own defaults (Planning choices made for
                # another configuration must not come back later by themselves)
                kw['selection'] = {}
        if topics is not None:
            clean = {}
            for role in ROLES:
                t = (topics or {}).get(role)
                if t and isinstance(t, str) and t.startswith('/') and len(t) < 200:
                    clean[role] = t
            kw['topics'] = clean
        if env is not None:
            kw['env'] = env or None
        if kw:
            s = self.reg.update_setup(robot_type, **kw)
            if 'mode' in kw and kw['mode'] != cur['mode']:
                self._event(robot_type, 'mode', 'Hardware configuration: %s.' % MODES[
                    kw['mode']]['label'], notify=False)
                for role in ROLES:   # arms that are no longer part of the setup are released
                    if role not in MODES[kw['mode']]['arms']:
                        self._stop_monitor(robot_type, role)
                if source == 'operator':
                    # what is there now was there when the operator chose: the monitor's next
                    # tick must not take it for a newly arrived arm (no auto-upgrade)
                    s2 = self.reg.setup(robot_type)
                    for role in ROLES:
                        self.prev[('ros', robot_type, role)] = self._ros_live(
                            robot_type, role, s2)[0]
                    if self._ports_seen:
                        self._tick_ports = {p['key'] for p in self.refresh_ports(force=True)}
        self.active_robot = robot_type
        return self.status(robot_type)

    def report_action_source(self, robot_type, supported, source=None):
        v = None if supported is None else bool(supported)
        self.action_source_supported[robot_type] = v
        if source:
            self.action_source_supported[(robot_type, source)] = v
        return {'ok': True}

    # ---- discovery ------------------------------------------------------------------
    def detect(self, robot_type):
        """Scan every free USB serial port for SO-101 motors (read-only)."""
        self.active_robot = robot_type
        ps = self.refresh_ports(force=True)
        spec = motor_spec(robot_type)
        results = {}

        def one(p):
            try:
                results[p['key']] = self._probe(robot_type, p)
            except HardwareError as e:
                results[p['key']] = {'ok': False, 'error': str(e), 'skipped': True,
                                     'device': p['device'], 'checked_at': time.time()}
                self.probes[p['key']] = results[p['key']]   # shown with the port

        threads = []
        for p in ps:
            # every port gets a result: a held / denied port is not opened, but its reason is
            # recorded so the operator sees why it was not scanned
            if spec:
                th = threading.Thread(target=one, args=(p,))
                th.start()
                threads.append(th)
        for th in threads:
            th.join(40)
        n_found = sum(1 for r in results.values() if r.get('ok'))
        self._event(robot_type, 'detect', 'Hardware scan: %d USB serial port%s, %d arm%s with '
                    'all motors answering.' % (len(ps), '' if len(ps) == 1 else 's', n_found,
                                               '' if n_found == 1 else 's'), notify=False)
        st = self.status(robot_type)
        st['scan'] = {'ports': len(ps), 'scanned': len(results), 'arms_ok': n_found,
                      'supported': bool(spec)}
        return st

    def assign(self, robot_type, key, role, label=None):
        if role not in ROLES:
            raise HardwareError('Role must be leader or follower.')
        port = next((p for p in self.refresh_ports(force=True) if p['key'] == key), None)
        if not port:
            raise HardwareError('That device is not plugged in any more.', 404)
        pr = self.probes.get(key)
        if not pr or time.time() - pr.get('checked_at', 0) > PROBE_TTL_S:
            if port['holders']:
                h = port['holders'][0]
                raise HardwareError('Validate the device first: RawBot Studio must scan its '
                                    'motors once, but %s (PID %d) holds the port. Stop it for a '
                                    'moment and press Detect Hardware.' % (h['name'], h['pid']),
                                    409)
            pr = self._probe(robot_type, port)
        if not pr.get('ok'):
            raise HardwareError('This device did not pass the motor check: %s' % (
                pr.get('error') or 'unknown'))
        other = self.reg.devices().get(key)
        dev = self.reg.assign(key, role, robot_type, port, pr, label)
        if other and other.get('role') != role:
            self._event(robot_type, 'assign', '%s is now the %s arm (was %s).' % (
                port['device'], role, other.get('role')))
        else:
            self._event(robot_type, 'assign', '%s registered as the %s arm (motor IDs %s).' % (
                port['device'], role, ', '.join(map(str, dev['motor_ids']))), notify=False)
        return self.status(robot_type)

    def unassign(self, robot_type, role):
        dev = self.reg.device_for(robot_type, role)
        if not dev:
            raise HardwareError('No %s arm is registered.' % role, 404)
        self._stop_monitor(robot_type, role)
        self.reg.unassign(dev['key'])
        self._event(robot_type, 'assign', 'The %s arm registration was removed.' % role,
                    notify=False)
        return self.status(robot_type)

    def configure(self, robot_type, role, label=None, port_override=None):
        dev = self.reg.device_for(robot_type, role)
        if not dev:
            raise HardwareError('Assign the %s arm first.' % role, 404)
        kw = {}
        if label is not None:
            kw['label'] = (label or '').strip()[:60] or dev['label']
        if port_override is not None:
            ov = (port_override or '').strip()
            if ov and not ov.startswith('/dev/') and not ov.upper().startswith('COM'):
                raise HardwareError('Port must be a device path such as /dev/serial/by-id/… '
                                    'or COM3.')
            kw['port_override'] = ov or None
        self.reg.touch(dev['key'], **kw)
        return self.status(robot_type)

    # ---- connection --------------------------------------------------------------------
    def connect(self, robot_type, role):
        setup = self.reg.setup(robot_type)
        if role not in MODES[setup['mode']]['arms']:
            raise HardwareError('The %s arm is not part of %s. Change the hardware '
                                'configuration first.' % (role, MODES[setup['mode']]['label']))
        disc = [r for r in setup['disconnected'] if r != role]
        self.reg.update_setup(robot_type, disconnected=disc)
        setup = self.reg.setup(robot_type)
        if self._monitor_status(robot_type, role).get('phase') == 'running':
            return self.status(robot_type)
        dev = self.reg.device_for(robot_type, role)
        port = self._port_for(dev) if dev else None
        topic = setup['topics'].get(role)
        ros_live = self._ros_live(robot_type, role, setup)[0]
        if port and not port['holders'] and not port.get('own_session') and \
                motor_spec(robot_type):
            pr = self._probe(robot_type, port)
            if not pr.get('ok'):
                raise HardwareError('%s arm: %s' % (role.capitalize(), pr.get('error')))
            self._start_monitor(robot_type, role, dev, port)
            t0 = time.time()
            while time.time() - t0 < 4.0:
                ph = self._monitor_status(robot_type, role).get('phase')
                if ph == 'running':
                    break
                if ph in ('failed', 'lost'):
                    break
                time.sleep(0.1)
            st = self._monitor_status(robot_type, role)
            if st.get('phase') != 'running':
                self._stop_monitor(robot_type, role)
                raise HardwareError('%s arm: the motors did not answer the read-only session '
                                    '(%s).' % (role.capitalize(), st.get('error') or
                                               st.get('phase')))
            self.reg.update_setup(robot_type, serial_wanted=sorted(set(
                setup.get('serial_wanted', [])) | {role}))
            self._event(robot_type, 'connect', '%s arm connected on %s (read-only).' % (
                role.capitalize(), port['device']), 'ok', notify=False)
        elif ros_live:
            self._event(robot_type, 'connect', '%s arm connected through bridge.py (%s).' % (
                role.capitalize(), topic), 'ok', notify=False)
        elif port and port['holders']:
            h = port['holders'][0]
            raise HardwareError('%s is in use by %s (PID %d), but no %s joint messages arrive on '
                                '%s. Check that bridge.py is running correctly.' % (
                                    port['device'], h['name'], h['pid'], role,
                                    topic or 'its ROS topic'), 409)
        elif not dev:
            raise HardwareError('No %s arm is set up%s. Plug it in and press Detect Hardware.'
                                % (role, '' if not topic else ' and no messages arrive on %s'
                                   % topic), 404)
        else:
            raise HardwareError('The %s arm is not plugged in (last seen on %s).' % (
                role, dev.get('last_port') or '?'), 404)
        return self.status(robot_type)

    def disconnect(self, robot_type, role):
        setup = self.reg.setup(robot_type)
        self._stop_monitor(robot_type, role)
        self.reg.update_setup(robot_type, disconnected=sorted(set(setup['disconnected']) |
                                                              {role}),
                              serial_wanted=[r for r in setup.get('serial_wanted', [])
                                             if r != role])
        self.ever_connected.pop((robot_type, role), None)
        self._event(robot_type, 'disconnect', '%s arm disconnected.' % role.capitalize(),
                    notify=False)
        return self.status(robot_type)

    def _start_monitor(self, robot_type, role, dev, port):
        self._stop_monitor(robot_type, role)
        py, env = self._python(robot_type)
        sid = 'mon-%s-%s' % (role, uuid.uuid4().hex[:6])
        d = self.sess_dir / sid
        d.mkdir(parents=True)
        cfg = {'port': port['device'], 'motors': motor_spec(robot_type),
               'status_file': str(d / 'status.json'), 'control_file': str(d / 'control.json'),
               'rate_hz': 30}
        atomic_write_json(d / 'config.json', cfg)
        atomic_write_json(d / 'control.json', {'command': 'run'})
        log = open(d / 'worker.log', 'ab', buffering=0)
        try:
            proc = subprocess.Popen([py, '-I', str(WORKER), 'monitor', str(d / 'config.json')],
                                    stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True, env=env)
        finally:
            log.close()
        with self.lock:
            self.monitors[(robot_type, role)] = {'proc': proc, 'dir': d, 'port': port['device'],
                                                 'key': dev['key'] if dev else port['key'],
                                                 'started': time.time()}
        self.ports_at = 0

    def _stop_monitor(self, robot_type, role, wait=2.0):
        with self.lock:
            m = self.monitors.pop((robot_type, role), None)
        if not m:
            return
        atomic_write_json(m['dir'] / 'control.json', {'command': 'stop', 't': time.time()})
        try:
            m['proc'].wait(wait)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(m['proc'].pid, signal.SIGKILL)
            except OSError:
                pass
        shutil.rmtree(m['dir'], ignore_errors=True)
        self.ports_at = 0

    def _monitor_status(self, robot_type, role):
        m = self.monitors.get((robot_type, role))
        if not m:
            return {}
        st = read_json(m['dir'] / 'status.json') or {}
        if m['proc'].poll() is not None and st.get('phase') == 'running':
            st['phase'] = 'lost'
            st['error'] = st.get('error') or 'the session ended unexpectedly'
        if st.get('phase') == 'running' and time.time() - float(st.get('t') or 0) > 1.5:
            st['phase'] = 'lost'
            st['error'] = 'no reading for %.1f s' % (time.time() - float(st.get('t') or 0))
        return st

    def _ros_live(self, robot_type, role, setup):
        """(live, age, names, values). A leader counts only if a program other than rosbridge
        publishes its topic (bridge.py); our own jog/rollout commands are not a leader."""
        topic = setup['topics'].get(role)
        if not topic:
            return False, None, [], []
        age, names, vals = self.ros.sample(topic)
        live = age is not None and age <= ROS_LIVE_S and bool(names)
        if live and (self.sim.owns(topic) or self.relay.owns(topic)):
            # the simulated robot publishes this topic: only another publisher is an arm
            pubs = self.ros.foreign_publishers(topic)
            live = bool(pubs)
        if live and role == 'leader':
            pubs = self.ros.foreign_publishers(topic)
            own_active = bool(self.jog and self.jog['proc'].poll() is None) or bool(
                self.rollouts and self.rollouts.active())
            if pubs is not None:
                live = bool(pubs)
            elif own_active:
                live = False
        return live, age, names, vals

    # ---- status ------------------------------------------------------------------------
    def status(self, robot_type):
        self.active_robot = robot_type or self.active_robot
        setup = self.reg.setup(robot_type)
        topics = {}
        for role, t in setup['topics'].items():
            topics[t] = ('trajectory_msgs/msg/JointTrajectory' if role == 'leader' else
                         'sensor_msgs/msg/JointState')
        cams = setup.get('cameras') or {}
        self.ros.watch(topics if setup['mode'] != 'simulation' else {},
                       publishers_of=[setup['topics'].get('follower')] + list(cams.values()),
                       frames=list(cams.values()) if self.sim.running else ())
        ps = self.refresh_ports()
        regs = self.reg.devices()
        arms = {r: self._arm(robot_type, r, setup, ps) for r in ROLES}
        unassigned = []
        for p in ps:
            dev = regs.get(p['key'])
            if dev and dev.get('robot_type') == robot_type:
                continue
            pr = self.probes.get(p['key'])
            unassigned.append({
                'key': p['key'], 'port': _port_view(p), 'registered_elsewhere':
                    dev and {'robot_type': dev.get('robot_type'), 'role': dev.get('role')},
                'probe': _probe_view(pr), 'holders': p['holders']})
        relay_on = self._update_relay(robot_type, setup, arms)
        extra = {'active_rollout': bool(self.rollouts and self.rollouts.active()),
                 'follower_relay': relay_on,
                 'jog_active': bool(self.jog and self.jog['proc'].poll() is None),
                 'calibration_active': bool(self.calib and self.calib['proc'].poll() is None),
                 'action_source_supported': self.action_source_supported.get(robot_type),
                 'command_topic': setup['topics'].get('leader'),
                 'command_publishers': self.ros.foreign_publishers(
                     setup['topics'].get('leader')) if setup['topics'].get('leader') else None}
        caps = capmod.compute(setup['mode'], arms, extra)
        plan = self._planning(robot_type, setup, arms, caps, extra)
        if plan['config'] == 'leader_sim':
            # demonstrations are recorded through the simulated follower (Planning decides)
            for k in ('record', 'collect'):
                caps[k] = dict(caps[k], available=plan['record']['available'],
                               reason=plan['record']['reason'])
        return {
            'robot_type': robot_type, 'mode': setup['mode'], 'mode_label': MODES[
                setup['mode']]['label'], 'mode_source': setup.get('mode_source'),
            'modes': [{'value': k, 'label': v['label'], 'arms': list(v['arms'])}
                      for k, v in MODES.items()],
            'topics': setup['topics'], 'env': setup.get('env'),
            'scan_supported': bool(motor_spec(robot_type)),
            'platform_ports': portmod.supported(),
            'arms': arms, 'unassigned': unassigned, 'capabilities': caps,
            'recording_gate': plan['record'],
            'planning': plan, 'sim': plan['sim'],
            'ros': {'connected': self.ros.connected, 'error': self.ros.error,
                    'url': self.ros.url},
            'sessions': {'calibration': self._calib_view(), 'jog': self._jog_view()},
            'events': list(self.events)[-30:],
            'event_seq': self.event_seq,
            'checked_at': now_iso(),
        }

    def auto_leader_sim(self, setup, arms):
        """Only a physical leader is there (Leader Only, or Leader + Follower with no follower
        ever set up): its simulated follower becomes the execution target by itself."""
        L, F = arms['leader'], arms['follower']
        return (setup['mode'] in ('leader_only', 'leader_follower') and
                L.get('state') == 'connected' and
                F.get('state') in ('unused', 'not_configured'))

    def _planning(self, robot_type, setup, arms, caps, extra):
        auto = self.auto_leader_sim(setup, arms)
        mode = 'leader_sim' if auto else setup['mode']
        ft = setup['topics'].get('follower')
        sel = setup.get('selection') or {}
        run = mode in ('simulation', 'leader_sim') or bool(setup.get('sim_twin')) or (
            sel.get('target') == 'follower' and sel.get('action') in planmod.SIM_LEADS +
            ('policy',))
        own_reason = ''
        if not ft:
            own_reason = 'The follower state topic of the robot configuration is not known yet.'
        elif 'follower' in MODES[mode]['arms']:
            own_reason = ('The configuration has a physical follower: its joint states are '
                          'the recorded state.')
        else:
            others = self.ros.foreign_publishers(ft)
            if others:
                own_reason = ('%s already publishes %s: stop it to record the simulated robot.'
                              % (', '.join(others), ft))
        names = setup.get('joints') or None
        if not names:
            for role in ('follower', 'leader'):
                t = setup['topics'].get(role)
                if t and not self.sim.owns(t):
                    n = self.ros.sample(t)[1]
                    if n:
                        names = n
                        break
        if not names:
            # the arms' own joints (serial motor table or their ROS messages)
            for role in ('follower', 'leader'):
                js = list((arms[role].get('joints') or {}).keys())
                if js and arms[role].get('state') == 'connected':
                    names = js
                    break
        # what this robot is (joints, gripper, limits, controls): from a registered custom
        # robot's URDF, else the robot configuration's joint order
        custom = robotcaps.custom_profile(self.robots_dir, robot_type)
        rc = robotcaps.capabilities(
            robot_type, names, custom,
            serial=bool(motor_spec(robot_type)), topics=setup['topics'])
        model, ws = self._kinematics(robot_type, setup, custom)
        rc['kinematics'] = bool(model)
        rc['workspace_guard'] = bool(model and ws)
        rc['joint_pairs'] = setup.get('joint_pairs') or {}
        # the leader read by RawBot Studio itself (serial session): converted to radians and
        # published for the recorder; with bridge.py the leader topic is used as before
        L = arms['leader']
        serial_leader = L.get('state') == 'connected' and L.get('source') == 'serial'
        # an arm whose own driver publishes its joints (JointState, e.g. a UFACTORY arm in
        # manual mode) is read the same way: the simulated follower copies it and its joints
        # are republished for the recorder as the action (leader_state)
        state_leader = L.get('state') == 'connected' and L.get('source') == 'ros' and \
            L.get('ros_kind') == 'state'
        sampler = self._leader_sampler(robot_type, setup) if serial_leader else \
            self._ros_leader_sampler(setup) if state_leader else None
        serial_leader = serial_leader or state_leader
        self.sim.ensure(robot_type, run and rc['sim']['available'], setup['topics'],
                        names=rc['names'] or names,
                        state_topic=ft if (run and not own_reason) else None,
                        leader_sampler=sampler, leader_pub=serial_leader,
                        limits=rc['limits'], pairs=setup.get('joint_pairs') or {},
                        kinematics={'model': model, 'workspace': ws})
        sim = self.sim.view()
        sim.update(can_own_state=not own_reason, own_reason=own_reason, twin=bool(
            setup.get('sim_twin')), joint_map=setup.get('joint_map') or {})
        compare = None
        if sim['running'] and ft and not self.sim.owns(ft) and \
                arms['follower'].get('state') == 'connected':
            compare = self.sim.compare(self.ros.sample(ft))
        cams = [{'name': k, 'topic': t, 'live': (None if self.ros.foreign_publishers(t) is None
                                                  else bool(self.ros.foreign_publishers(t)))}
                for k, t in sorted((setup.get('cameras') or {}).items())]
        # simulated cameras: when the simulated robot is the recorded robot, a configured
        # camera that no real node publishes is rendered from the simulation by a RawBot
        # Studio page (sim/SimCameraPublisher.js). Live only once its frames arrive.
        sim_cams = []
        if model and sim['running'] and sim.get('state_topic') and \
                setup.get('sim_cameras') is not False:
            for c in cams:
                if c['live'] is False:
                    age = self.ros.age(c['topic'])
                    c.update(sim=True, live=age is not None and age < 2.5)
                    sim_cams.append({'name': c['name'], 'topic': c['topic'], 'view': 'wrist' if
                                     re.search(r'wrist|grip|hand|tool', c['name'], re.I)
                                     else 'scene'})
        if model:
            sim['chain'] = {'joints': model.chain, 'names': model.joint_of}
        out = planmod.options(mode, arms, caps, sim, setup.get('selection'), cams, {
            'jog_running': extra.get('jog_active'), 'compare': compare,
            'calibration_active': extra.get('calibration_active'),
            'follower_relay': extra.get('follower_relay'),
            'pairs': setup.get('joint_pairs') or {},
            'rollout': _rollout_view(self.rollouts.active() if self.rollouts else None),
            'jog_held': (self._jog_view() or {}).get('held_reason'),
            'leader_state_supported': self.action_source_supported.get(
                (robot_type, 'leader_state'))}, robot=rc, auto=auto)
        # what drives the simulated robot follows the selection: the leader arm, a mirror of
        # the physical follower (physical-to-simulation), or the operator (manual / script /
        # replay, started from the Planning page)
        key = (robot_type, out['selection'].get('action'), out['selection'].get('target'),
               sim['running'])
        changed, self._sim_key = key != getattr(self, '_sim_key', None), key
        if sim['running'] and changed:     # only when the choice changes, not on every poll
            want = planmod.sim_drive(out['selection'])
            cur = self.sim.drive
            if want and cur != want or (not want and cur in ('leader', 'follower')):
                try:
                    self.sim.command({'command': 'drive', 'drive': want or 'manual'})
                except ValueError:
                    pass
                sim = dict(sim, **self.sim.view())
        out['sim'] = sim
        out['sim_cameras'] = sim_cams
        return out

    def _kinematics(self, robot_type, setup, custom=None):
        """(model, workspace limits) of the robot: its URDF or the SO-101 chain, and the
        planning workspace (floor only until the robot configuration sent one). The guard can
        be switched off per robot (``workspace_guard``)."""
        if custom is None:
            custom = robotcaps.custom_profile(self.robots_dir, robot_type)
        model = kinmod.model_for(robot_type, (custom or {}).get('urdf_joints'),
                                 (custom or {}).get('urdf_to_ros'), bool(motor_spec(robot_type)))
        if not model or setup.get('workspace_guard') is False:
            return model, None
        return model, kinmod.workspace_from_profile(setup.get('workspace') or {})

    def _update_relay(self, robot_type, setup, arms):
        """Publish the follower's joint states from the read-only serial session when the
        configuration has a follower, RawBot Studio reads it and nothing else publishes its
        state topic (bridge.py stopped)."""
        F = arms['follower']
        ft = setup['topics'].get('follower')
        on = bool('follower' in MODES[setup['mode']]['arms'] and ft and
                  F.get('state') == 'connected' and F.get('source') == 'serial' and
                  not self.ros.foreign_publishers(ft))
        if on:
            self.relay.set(ft, self._arm_sampler(robot_type, setup, 'follower'))
            # only claim publishing once messages actually went out over rosbridge
            v = self.relay.view() or {}
            if v.get('error'):
                F['publish_error'] = v['error']
            elif v.get('published'):
                F['publishes'] = ft
            else:
                F['publish_pending'] = ft
        elif self.relay.topic:
            self.relay.clear()
        return on

    def _leader_sampler(self, robot_type, setup):
        return self._arm_sampler(robot_type, setup, 'leader')

    def _ros_leader_sampler(self, setup):
        """Leader joints from the arm driver's JointState topic (already in radians)."""
        topic = setup['topics'].get('leader')

        def sample():
            age, names, vals = self.ros.sample(topic)
            if age is None:
                return None, [], [], {}
            return age, names, vals, {'source': 'ros', 'topic': topic}
        return sample

    def _arm_sampler(self, robot_type, setup, role):
        """Leader joints from RawBot Studio's read-only serial session, in radians: raw
        Present_Position ticks of the calibrated motors (LeRobot half-turn homing puts the
        middle of each range at 2048) -> (ticks - 2048) * 2*pi / 4096, then the joint map's
        sign and offset."""
        jm = setup.get('joint_map') or {}

        def sample():
            mon = self._monitor_status(robot_type, role)
            pos = mon.get('positions') or {}
            if mon.get('phase') != 'running' or not pos:
                return None, [], [], {}
            names = list(pos)
            vals = [ticks_to_rad(pos[n], jm.get(n)) for n in names]
            return (time.time() - float(mon.get('t') or 0), names, vals,
                    {'source': 'serial', 'ticks': [int(pos[n]) for n in names],
                     'rate_hz': mon.get('rate_hz')})
        return sample

    def hybrid(self, robot_type, on):
        """Hybrid control: the operator takes over from the running policy (the simulated
        leader, lined up with the arm first, drives it through the rollout's own safety filter)
        or hands back. One command owner - the rollout worker - throughout."""
        r = self.rollouts.active() if self.rollouts else None
        if not r:
            raise HardwareError('No policy is running.', 409)
        setup = self.reg.setup(robot_type)
        if on:
            if not self.sim.running:
                raise HardwareError('The simulated leader is not running.', 409)
            age, fnames, fvals = self.ros.sample(setup['topics'].get('follower'))
            if age is None or age > 1.0:
                raise HardwareError('No fresh follower joint states.', 409)
            self.sim.command({'command': 'set_pose', 'pose': dict(zip(fnames, fvals))})
        try:
            self.rollouts.takeover(r['id'], on)
        except ValueError as e:
            raise HardwareError(str(e), 409)
        if not on and self.sim.running:
            try:
                self.sim.command({'command': 'drive', 'drive': 'follower'})
            except ValueError:
                pass
        self._event(robot_type, 'hybrid', 'Operator took over from the policy.' if on else
                    'Control handed back to the policy.', notify=False)
        return self.status(robot_type)

    def sim_command(self, robot_type, body):
        if robot_type and robot_type != self.sim.robot_type:
            self.status(robot_type)
        if body.get('command') == 'replay' and body.get('capture'):
            # a saved trajectory capture (leader arm or simulated robot) replayed in simulation
            rec = self.reg.capture(str(body['capture']))
            if not rec:
                raise HardwareError('Capture %s not found.' % body['capture'], 404)
            body = dict(body, names=rec['names'], t=[s[0] for s in rec['samples']],
                        rows=[s[1:] for s in rec['samples']],
                        label=body.get('label') or 'capture %s' % rec['id'])
        if body.get('command') == 'capture_stop':
            got = self.sim.take_capture()
            if not got:
                raise HardwareError('No trajectory was captured.')
            saved = self.reg.save_capture(robot_type or self.sim.robot_type, 'sim', got[0],
                                          got[1], 'sim')
            out = self.status(robot_type or self.active_robot)
            out['saved_capture'] = saved
            return out
        try:
            self.sim.command(body)
        except ValueError as e:
            raise HardwareError(str(e))
        return self.status(robot_type or self.active_robot)

    def _arm(self, robot_type, role, setup, ps):
        used = role in MODES[setup['mode']]['arms']
        dev = self.reg.device_for(robot_type, role)
        port = None
        if dev:
            port = next((p for p in ps if p['key'] == dev['key']), None)
            if not port and dev.get('port_override'):
                port = next((p for p in ps if dev['port_override'] in (p['device'],
                                                                       p.get('by_id'))), None)
        mon = self._monitor_status(robot_type, role)
        mon_running = mon.get('phase') == 'running'
        live, age, names, vals = self._ros_live(robot_type, role, setup)
        pr = self.probes.get(dev['key']) if dev else None
        spec = motor_spec(robot_type) or []
        card = {'role': role, 'used': used, 'device': dev and {
                    k: dev.get(k) for k in ('key', 'label', 'vid', 'pid', 'serial', 'product',
                                            'unique', 'motor_ids', 'last_port', 'port_override',
                                            'last_validated_at')},
                'port': _port_view(port) if port else None,
                'holders': (port or {}).get('holders') or [],
                'topic': setup['topics'].get(role), 'source': None, 'detail': ''}
        k = (robot_type, role)
        disconnected = role in setup['disconnected']
        calib_here = self.calib and self.calib['proc'].poll() is None and \
            self.calib['role'] == role and self.calib['robot_type'] == robot_type
        if not used:
            state, detail = 'unused', 'Not part of %s.' % MODES[setup['mode']]['label']
        elif calib_here:
            state, detail = 'calibrating', 'Calibration in progress on %s.' % self.calib['port']
        elif disconnected:
            state, detail = 'disconnected', 'Disconnected in RawBot Studio.'
        elif mon_running:
            state, detail = 'connected', 'Read-only session on %s.' % (
                (self.monitors.get(k) or {}).get('port'))
            card['source'] = 'serial'
        elif k in self.monitors and mon.get('phase') in (None, 'connecting') and \
                self.monitors[k]['proc'].poll() is None:
            state, detail = 'connecting', 'Opening %s read-only.' % self.monitors[k]['port']
        elif k in self.monitors:   # session exists but stopped answering
            state, detail = 'lost', mon.get('error') or 'The motors stopped answering.'
        elif live:
            card['source'] = 'ros'
            card['ros_kind'] = self.ros.kind(card['topic'])
            state, detail = 'connected', (
                'The arm\'s driver publishes its joints on %s.' % card['topic']
                if card['ros_kind'] == 'state' else 'bridge.py publishes %s.' % card['topic'])
        elif self.ever_connected.get(k) and (not dev or port is None or port['holders']):
            state = 'lost'
            detail = ('Unplugged.' if dev and port is None else
                      'No %s messages for %.1f s.' % (role, age) if age else
                      'No %s messages.' % role)
        elif dev and port and port['holders']:
            h = port['holders'][0]
            state = 'in_use'
            detail = '%s is held by %s (PID %d) but no joint messages arrive on %s.' % (
                port['device'], h['name'], h['pid'], card['topic'] or 'ROS')
        elif dev and port:
            if pr and not pr.get('ok') and not pr.get('skipped'):
                state, detail = 'error', pr.get('error') or 'Motor check failed.'
            else:
                state, detail = 'detected', 'Plugged in on %s.' % port['device']
        elif dev:
            state, detail = 'not_found', 'Not plugged in (last seen on %s).' % (
                dev.get('last_port') or '?')
        elif not spec:
            # no motor table: this robot's arms are only reached through ROS, so the fix is a
            # topic in its robot configuration or the arm's driver, not a USB scan
            state = 'not_configured'
            card['ros_only'] = True
            detail = ('No %s topic in the robot configuration (Robots -> Robot setup -> '
                      'Verify -> Robot configuration).' % role if not card['topic'] else
                      'No joint messages on %s. Start the arm\'s driver.' % card['topic'])
        else:
            state, detail = 'not_configured', 'No %s arm set up.' % role
        if state == 'connected':
            self.ever_connected[k] = True
        card['state'] = state
        card['detail'] = detail
        # ---- motors / joints / calibration
        expected = [m[1] for m in spec]
        joints = {}
        if mon_running:
            pos = mon.get('positions') or {}
            for name, mid, _ in spec:
                joints[name] = {'id': mid, 'comm': 'ok' if (mon.get('ok') or {}).get(name)
                                else 'fail', 'value': pos.get(name), 'units': 'ticks'}
            card['live'] = {'age_s': round(time.time() - float(mon.get('t') or 0), 2),
                            'rate_hz': mon.get('rate_hz'), 'source': 'serial'}
        elif live or (card['topic'] and age is not None):
            vm = dict(zip(names, vals))
            ok = age is not None and age <= ROS_LIVE_S
            for n in names:
                joints[n] = {'comm': 'ok' if ok else 'stale', 'value': vm.get(n), 'units': 'ROS'}
            card['live'] = {'age_s': round(age, 2) if age is not None else None,
                            'source': 'ros'}
        elif pr and pr.get('joints'):
            for name, j in pr['joints'].items():
                joints[name] = {'id': j.get('id'), 'comm': j.get('comm'),
                                'value': j.get('position'), 'units': 'ticks',
                                'error': j.get('error'), 'from_scan': True}
        card['joints'] = joints
        found = sorted(int(i) for i in (pr or {}).get('found', {})) if pr else None
        card['motors'] = {'expected': expected, 'found': found,
                          'missing': (pr or {}).get('missing_ids') if pr else None,
                          'checked_at': pr.get('checked_at') if pr else None,
                          'registered': (dev or {}).get('motor_ids')}
        card['calibration'] = self._calibration_view(robot_type, role, dev, pr)
        # ---- what this card can do now
        free = bool(port and not port['holders'] and not port.get('own_session'))
        if not used:
            cr = 'Not part of the current configuration.'
        elif not spec:
            cr = 'Calibration from RawBot Studio is available for SO-101 arms.'
        elif not dev:
            cr = 'Assign the %s arm first (Detect Hardware).' % role
        elif not port:
            cr = 'The %s arm is not plugged in.' % role
        elif port['holders']:
            h = port['holders'][0]
            cr = 'Stop %s (PID %d) first: calibration needs the port.' % (h['name'], h['pid'])
        elif self.calib and self.calib['proc'].poll() is None:
            cr = 'Another calibration is running.'
        elif port.get('own_session') == 'monitor':
            cr = None   # the read-only session is closed automatically for calibration
        else:
            cr = None
        card['can_calibrate'] = cr is None
        card['calibrate_reason'] = cr
        card['can_connect'] = used and state not in ('connected', 'calibrating')
        card['can_disconnect'] = state in ('connected', 'lost', 'in_use', 'detected')
        card['port_free'] = free
        return card

    def _calibration_view(self, robot_type, role, dev, pr):
        profs = self.reg.profiles(robot_type, role)
        cal = (pr or {}).get('calibration')
        view = {'state': (pr or {}).get('calibration_state') or 'unknown', 'profile': None,
                'profiles': [{k: p.get(k) for k in ('id', 'name', 'created_at', 'device_key',
                                                    'source')} for p in profs[:10]]}
        if cal:
            for p in profs:
                pm = p.get('motors') or {}
                if pm and all(n in cal and all(int(pm[n][f]) == int(cal[n][f]) for f in (
                        'homing_offset', 'range_min', 'range_max')) for n in pm):
                    view['profile'] = {'id': p['id'], 'name': p['name']}
                    break
        if view['state'] == 'calibrated':
            view['text'] = ('Calibrated - matches "%s"' % view['profile']['name']
                            if view['profile'] else 'Calibrated on the motors (no saved profile '
                            'matches)')
        elif view['state'] == 'factory':
            view['text'] = 'Not calibrated (factory values)'
        else:
            view['text'] = ('Unknown - scan the motors while bridge.py is stopped' if dev
                            else 'Unknown')
        return view

    # ---- monitor loop --------------------------------------------------------------------
    def _monitor_loop(self):
        while not self._stop:
            try:
                rt = self.active_robot
                if rt:
                    self._tick(rt)
            except Exception as e:  # noqa: BLE001
                self.log('hardware monitor: %s' % e)
            time.sleep(1.5)

    def _tick(self, robot_type):
        prev_ports = self._tick_ports   # keys seen by the previous tick (not by status())
        first = not self._ports_seen
        ps = self.refresh_ports(force=True)
        self._ports_seen = True
        now_ports = {p['key'] for p in ps}
        self._tick_ports = now_ports
        setup = self.reg.setup(robot_type)
        regs = {d['key']: d for d in (self.reg.device_for(robot_type, r) for r in ROLES) if d}
        appeared = set()
        busy = bool(self.calib and self.calib['proc'].poll() is None)
        for key, dev in regs.items():
            role = dev['role']
            if first:
                continue
            if key in prev_ports and key not in now_ports:
                self._event(robot_type, 'unplug', '%s arm unplugged.' % role.capitalize(),
                            'warn')
            if key in now_ports and key not in prev_ports:
                port = next(p for p in ps if p['key'] == key)
                self.reg.touch(key, last_port=port['device'])
                self._event(robot_type, 'plug', '%s arm plugged in on %s.' % (
                    role.capitalize(), port['device']), 'info', notify=False)
                # validate the identity right away (read-only) when nobody holds the port
                if not port['holders'] and not port.get('own_session') and not busy and \
                        motor_spec(robot_type):
                    try:
                        if self._probe(robot_type, port).get('ok'):
                            appeared.add(role)
                    except HardwareError:
                        pass
        # recovery: re-open a read-only session the operator wanted, once the arm is back
        for role in setup.get('serial_wanted', []):
            k = (robot_type, role)
            if role in setup['disconnected'] or role not in MODES[setup['mode']]['arms']:
                continue
            dev = self.reg.device_for(robot_type, role)
            if not dev:
                continue
            port = next((p for p in ps if p['key'] == dev['key']), None)
            st = self._monitor_status(robot_type, role)
            if k in self.monitors and (st.get('phase') in ('lost', 'failed') or
                                       self.monitors[k]['proc'].poll() is not None):
                if port is None or port['device'] != self.monitors[k]['port'] or \
                        self.monitors[k]['proc'].poll() is not None:
                    self._stop_monitor(robot_type, role, wait=0.5)
            if k not in self.monitors and port and not port['holders'] and not busy:
                try:
                    pr = self.probes.get(port['key'])
                    if not pr or time.time() - pr.get('checked_at', 0) > 5:
                        pr = self._probe(robot_type, port)
                    if pr.get('ok'):
                        self._start_monitor(robot_type, role, dev, port)
                        self._event(robot_type, 'reconnect', '%s arm reconnected on %s.' % (
                            role.capitalize(), port['device']), 'ok')
                except HardwareError:
                    pass
        # state transitions -> events
        st = self.status(robot_type)
        for role in ROLES:
            k = (robot_type, role)
            cur = st['arms'][role]['state']
            old = self.prev.get(k)
            if old and old != cur:
                if cur == 'lost':
                    self._event(robot_type, 'lost', '%s arm lost: %s' % (
                        role.capitalize(), st['arms'][role]['detail']), 'err')
                elif cur == 'connected' and old == 'lost':
                    self._event(robot_type, 'reconnect', '%s arm is back.' % role.capitalize(),
                                'ok')
                elif cur == 'connected':
                    self._event(robot_type, 'connect', '%s arm connected (%s).' % (
                        role.capitalize(), 'bridge.py' if st['arms'][role]['source'] == 'ros'
                        else 'read-only session'), 'ok', notify=False)
            self.prev[k] = cur
        # the arm that was missing shows up (only through ROS or a fresh plug-in)
        for role in ROLES:
            k = ('ros', robot_type, role)
            live = self._ros_live(robot_type, role, setup)[0]
            if live and self.prev.get(k) is False:
                appeared.add(role)
            self.prev[k] = live
        if not first:
            self._auto_upgrade(robot_type, st['mode'], appeared)

    def _auto_upgrade(self, robot_type, mode, appeared):
        """Started with one arm and the other one shows up: enable Leader + Follower without a
        new project. Only a NEW arrival counts (plugged in and validated, or bridge.py starts
        publishing it), so an arm that was already there when the operator chose a one-arm
        setup does not override that choice. Simulation Only is never changed."""
        if mode not in ('leader_only', 'follower_only'):
            return
        other = 'follower' if mode == 'leader_only' else 'leader'
        if other in appeared:
            self.setup(robot_type, mode='leader_follower', source='auto')
            self._event(robot_type, 'mode', '%s arm detected - Leader + Follower is now active.'
                        % other.capitalize(), 'ok')

    # ---- calibration -----------------------------------------------------------------------
    def calibration_start(self, robot_type, role, mode='calibrate', profile_id=None):
        if self.calib and self.calib['proc'].poll() is None:
            raise HardwareError('A calibration is already running (%s).' % self.calib['role'],
                                409)
        if self.jog and self.jog['proc'].poll() is None:
            raise HardwareError('Stop the jog session first.', 409)
        if self.rollouts and self.rollouts.active():
            raise HardwareError('A policy rollout is running.', 409)
        setup = self.reg.setup(robot_type)
        st = self.status(robot_type)
        card = st['arms'][role]
        if not card['can_calibrate']:
            raise HardwareError(card['calibrate_reason'] or 'Calibration is not possible now.',
                                409)
        dev = self.reg.device_for(robot_type, role)
        # the read-only session (if any) gives the port to the calibration
        restart_monitor = (robot_type, role) in self.monitors
        self._stop_monitor(robot_type, role)
        port = self._port_for(dev)
        if not port or port['holders']:
            raise HardwareError('The %s port is not free.' % role, 409)
        prof = None
        if mode == 'apply':
            prof = self.reg.profile(profile_id)
            if prof.get('role') != role or prof.get('robot_type') != robot_type:
                raise HardwareError('That profile belongs to the %s of %s.' % (
                    prof.get('role'), prof.get('robot_type')))
        py, env = self._python(robot_type)
        cid = 'cal-%s-%s' % (role, uuid.uuid4().hex[:6])
        d = self.sess_dir / cid
        d.mkdir(parents=True)
        cfg = {'port': port['device'], 'motors': motor_spec(robot_type), 'mode': mode,
               'profile': prof and prof['motors'], 'status_file': str(d / 'status.json'),
               'control_file': str(d / 'control.json'), 'heartbeat_timeout_s': 15}
        atomic_write_json(d / 'config.json', cfg)
        atomic_write_json(d / 'control.json', {'command': None, 'heartbeat': time.time()})
        log = open(d / 'worker.log', 'ab', buffering=0)
        try:
            proc = subprocess.Popen([py, '-I', str(WORKER), 'calibrate', str(d / 'config.json')],
                                    stdout=log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True, env=env)
        finally:
            log.close()
        self.calib = {'id': cid, 'proc': proc, 'dir': d, 'role': role, 'robot_type': robot_type,
                      'port': port['device'], 'key': dev['key'], 'mode': mode,
                      'profile_id': profile_id, 'started': time.time(), 'saved': None,
                      'restart_monitor': restart_monitor, 'setup_mode': setup['mode']}
        self.ports_at = 0
        self._event(robot_type, 'calibration', '%s calibration started on %s.' % (
            role.capitalize(), port['device']), notify=False)
        return self._calib_view()

    def calibration_command(self, cid, command, name=None):
        c = self.calib
        if not c or c['id'] != cid:
            raise HardwareError('Calibration session not found.', 404)
        if command not in ('begin', 'set_middle', 'finish', 'cancel', 'heartbeat'):
            raise HardwareError('Unknown calibration command.')
        ctl = read_json(c['dir'] / 'control.json') or {}
        ctl['heartbeat'] = time.time()
        if command != 'heartbeat':
            ctl.update(command=command, t=time.time())
            if name:
                c['name'] = name[:80]
        atomic_write_json(c['dir'] / 'control.json', ctl)
        if command in ('finish', 'cancel', 'begin'):
            t0 = time.time()
            while time.time() - t0 < 3.0:
                ph = (read_json(c['dir'] / 'status.json') or {}).get('phase')
                if (command == 'cancel' and ph in ('cancelled', 'failed')) or \
                        (command == 'finish' and ph in ('done', 'failed', 'cancelled')) or \
                        (command == 'begin' and ph in ('torque_off', 'done', 'failed')):
                    break
                if command == 'finish' and (read_json(c['dir'] / 'status.json') or {}).get(
                        'warning'):
                    break
                time.sleep(0.05)
        return self._calib_view()

    def _calib_view(self):
        c = self.calib
        if not c:
            return None
        st = read_json(c['dir'] / 'status.json') or {}
        running = c['proc'].poll() is None
        ph = st.get('phase') or ('starting' if running else 'failed')
        if not running and ph not in ('done', 'cancelled', 'failed'):
            ph = 'failed'
            st['error'] = st.get('error') or 'The calibration worker exited (code %s).' % (
                c['proc'].returncode)
        if ph == 'done' and not c.get('saved') and st.get('result'):
            prof = self.reg.save_profile(c['role'], c['robot_type'], c['key'], st['result'],
                                         name=c.get('name'),
                                         source='applied profile' if c['mode'] == 'apply'
                                         else 'calibration')
            c['saved'] = prof['id']
            key = c['key']
            if key in self.probes:   # the motors now carry these values
                self.probes[key]['calibration'] = st['result']
                self.probes[key]['calibration_state'] = 'calibrated'
            self._event(c['robot_type'], 'calibration', '%s calibration %s and saved as "%s".'
                        % (c['role'].capitalize(), 'applied' if c['mode'] == 'apply'
                           else 'finished', prof['name']), 'ok')
        if ph in ('cancelled', 'failed') and not c.get('reported'):
            c['reported'] = True
            self._event(c['robot_type'], 'calibration', '%s calibration %s: %s' % (
                c['role'].capitalize(), ph, st.get('end_reason') or st.get('error') or ''),
                'warn')
        if not running and c.get('restart_monitor') and not c.get('monitor_restored'):
            c['monitor_restored'] = True
            dev = self.reg.device_for(c['robot_type'], c['role'])
            port = self._port_for(dev) if dev else None
            if port and not port['holders']:
                try:
                    self._start_monitor(c['robot_type'], c['role'], dev, port)
                except HardwareError:
                    pass
        return {'id': c['id'], 'role': c['role'], 'robot_type': c['robot_type'],
                'port': c['port'], 'mode': c['mode'], 'phase': ph, 'running': running,
                'positions': st.get('positions'), 'mins': st.get('mins'),
                'maxes': st.get('maxes'), 'names': st.get('names'), 'warning': st.get('warning'),
                'error': st.get('error'), 'end_reason': st.get('end_reason'),
                'restored': st.get('restored'), 'restore_error': st.get('restore_error'),
                'saved_profile': c.get('saved'), 'written': st.get('written')}

    def calibration_dismiss(self):
        if self.calib and self.calib['proc'].poll() is None:
            raise HardwareError('The calibration is still running; cancel it first.', 409)
        self.calib = None
        return {'ok': True}

    def install_profile(self, pid, lerobot_id):
        """Copy a profile to LeRobot's calibration folder (the file `lerobot-*` commands and a
        LeRobot-based bridge.py read). An existing file is kept as a .bak copy."""
        prof = self.reg.profile(pid)
        lid = (lerobot_id or '').strip()
        if not lid or not all(c.isalnum() or c in '-_.' for c in lid):
            raise HardwareError('Give the LeRobot id of the arm (letters, digits, - _ .).')
        base = Path(os.environ.get('HF_LEROBOT_CALIBRATION') or os.path.join(
            os.environ.get('HF_LEROBOT_HOME') or os.path.expanduser(
                '~/.cache/huggingface/lerobot'), 'calibration'))
        sub = ('robots/so101_follower' if prof['role'] == 'follower'
               else 'teleoperators/so101_leader')
        target = base / sub / ('%s.json' % lid)
        target.parent.mkdir(parents=True, exist_ok=True)
        backup = None
        if target.exists():
            backup = target.with_suffix('.json.bak-%s' % time.strftime('%Y%m%d-%H%M%S'))
            shutil.copy2(target, backup)
        data = {n: {k: int(v[k]) for k in ('id', 'drive_mode', 'homing_offset', 'range_min',
                                             'range_max')} for n, v in prof['motors'].items()}
        with open(target, 'w') as f:
            json.dump(data, f, indent=4)
        return {'path': str(target), 'backup': str(backup) if backup else None}

    # ---- jog --------------------------------------------------------------------------------
    def jog_prepare(self, robot_type, req):
        """Check everything and return the exact session to authorise."""
        st = self.status(robot_type)
        cap = st['capabilities']['follower_control']
        errors = [] if cap['available'] else [cap['reason']]
        setup = self.reg.setup(robot_type)
        names = [n for n in (req.get('names') or []) if isinstance(n, str)]
        if not names:
            errors.append('The robot configuration has no joint order for the command topic.')
        limits = []
        for j in req.get('limits') or []:
            try:
                lo, hi = float(j['min']), float(j['max'])
                spd = float(j.get('speed') or 0)
            except (KeyError, TypeError, ValueError):
                errors.append('Limits must be numbers.')
                break
            if lo >= hi:
                errors.append('%s: minimum must be below maximum.' % j.get('name'))
            span = hi - lo
            if spd <= 0 or spd > 0.25 * span:
                errors.append('%s: speed must be > 0 and at most 25%% of the range per second '
                              '(%.4f).' % (j.get('name'), 0.25 * span))
            limits.append({'name': j.get('name'), 'min': lo, 'max': hi, 'speed': spd,
                           'max_lead': round(max(spd * 0.3, 0.02 * span), 5)})
        if [j['name'] for j in limits] != names:
            errors.append('Limits must list exactly the commanded joints, in order.')
        if self.calib and self.calib['proc'].poll() is None:
            errors.append('A calibration is running.')
        cfg = {'robot_type': robot_type, 'state_topic': setup['topics'].get('follower'),
               'command_topic': setup['topics'].get('leader'), 'names': names,
               'limits': limits, 'rosbridge_url': self.ros.url,
               'max_duration_s': 900, 'rate_hz': 30}
        model, ws = self._kinematics(robot_type, setup)
        if model and ws:
            cfg['kinematics'] = {'joints': model.chain, 'names': model.joint_of, 'workspace': ws}
        if req.get('follow'):
            # virtual teleoperation: the physical follower follows the simulated leader
            sv = self.sim.view()
            if not sv['running']:
                errors.append('The simulated leader is not running.')
            pairs = setup.get('joint_pairs') or {}
            fmap = {n: pairs.get(n, n) for n in names}
            miss = [n for n in names if fmap[n] not in sv['names']]
            if miss:
                errors.append('The simulated leader has no joint for %s.' % ', '.join(miss))
            cfg['follow'] = {'topic': SIM_TOPIC, 'map': fmap}
        token = _token(cfg)
        return {'ok': not errors, 'errors': errors, 'config': cfg, 'token': token,
                'expires_in_s': 120}

    def jog_start(self, robot_type, req, token, confirmations):
        need = ('area_clear', 'estop_ready', 'watching')
        if not all((confirmations or {}).get(k) for k in need):
            raise HardwareError('Confirm the safety checklist first.')
        prep = self.jog_prepare(robot_type, req)
        if not prep['ok']:
            raise HardwareError(' '.join(prep['errors']), 409)
        if token != prep['token']:
            raise HardwareError('The jog settings changed since you reviewed them.', 409)
        if self.jog and self.jog['proc'].poll() is None:
            raise HardwareError('A jog session is already running.', 409)
        if prep['config'].get('follow'):
            # start where the arm is: the simulated leader takes the follower's pose, so
            # switching following on never makes the arm jump
            age, fnames, fvals = self.ros.sample(prep['config']['state_topic'])
            if age is None or age > 1.0:
                raise HardwareError('No fresh follower joint states to line the simulated '
                                    'leader up with.', 409)
            fmap = prep['config']['follow']['map']
            pose = {fmap[n]: v for n, v in zip(fnames, fvals) if n in fmap}
            try:
                self.sim.command({'command': 'set_pose', 'pose': pose})
            except ValueError as e:
                raise HardwareError(str(e), 409)
        jid = 'jog-%s' % uuid.uuid4().hex[:6]
        d = self.sess_dir / jid
        d.mkdir(parents=True)
        cfg = dict(prep['config'], status_file=str(d / 'status.json'),
                   control_file=str(d / 'control.json'))
        atomic_write_json(d / 'config.json', cfg)
        atomic_write_json(d / 'control.json', {'command': 'run', 'heartbeat': time.time()})
        log = open(d / 'worker.log', 'ab', buffering=0)
        try:
            proc = subprocess.Popen([sys.executable, '-I', str(JOG_WORKER),
                                     str(d / 'config.json')], stdout=log,
                                    stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                    start_new_session=True)
        finally:
            log.close()
        self.jog = {'id': jid, 'proc': proc, 'dir': d, 'robot_type': robot_type,
                    'follow': bool(prep['config'].get('follow')),
                    'started': time.time()}
        self._event(robot_type, 'jog', 'Direct control session started.', notify=False)
        return self._jog_view()

    def jog_command(self, jid, body):
        # the page's 100 ms heartbeats and Stop arrive on parallel requests: one at a time, so
        # a heartbeat can never write back over a Stop
        with self._jog_lock:
            return self._jog_command(jid, body)

    def _jog_command(self, jid, body):
        j = self.jog
        if not j or j['id'] != jid:
            raise HardwareError('Jog session not found.', 404)
        ctl = read_json(j['dir'] / 'control.json') or {}
        if ctl.get('command') in ('stop', 'estop') and body.get('command') not in (
                'stop', 'estop'):
            return self._jog_view()        # stopping: moves and heartbeats are ignored
        ctl['heartbeat'] = time.time()
        cmd = body.get('command')
        if cmd == 'move':
            d = int(body.get('dir') or 0)
            ctl['move'] = {'joint': body.get('joint'), 'dir': d if d in (-1, 1) else 0,
                           't': time.time()}
        elif cmd == 'follow':
            on = bool(body.get('on'))
            if on and not (ctl.get('follow') or {}).get('on') and j.get('follow'):
                # switching on: the simulated leader starts from where the arm is now, so the
                # arm never jumps to an old simulated pose
                cfg = read_json(j['dir'] / 'config.json') or {}
                age, fnames, fvals = self.ros.sample(cfg.get('state_topic'))
                if age is None or age > 1.0:
                    raise HardwareError('No fresh follower joint states.', 409)
                fmap = (cfg.get('follow') or {}).get('map') or {}
                try:
                    self.sim.command({'command': 'set_pose', 'pose': {
                        fmap[n]: v for n, v in zip(fnames, fvals) if n in fmap}})
                except ValueError as e:
                    raise HardwareError(str(e), 409)
            ctl['follow'] = {'on': on, 't': time.time()}
        elif cmd in ('release', 'heartbeat'):
            if cmd == 'release':
                ctl['move'] = {'joint': None, 'dir': 0, 't': time.time()}
        elif cmd in ('stop', 'estop'):
            ctl['command'] = cmd
            ctl['move'] = {'joint': None, 'dir': 0, 't': time.time()}
        else:
            raise HardwareError('Unknown jog command.')
        atomic_write_json(j['dir'] / 'control.json', ctl)
        if cmd == 'estop':
            threading.Thread(target=self._kill_later, args=(j['proc'],), daemon=True).start()
        return self._jog_view()

    def _jog_view(self):
        j = self.jog
        if not j:
            return None
        st = read_json(j['dir'] / 'status.json') or {}
        running = j['proc'].poll() is None
        ph = st.get('phase') or ('starting' if running else 'failed')
        if not running and ph not in ('stopped', 'failed'):
            ph = 'failed' if j['proc'].returncode not in (0, None, -9) else 'stopped'
        return {'id': j['id'], 'running': running, 'phase': ph, 'error': st.get('error'),
                'end_reason': st.get('end_reason'), 'measured': st.get('measured'),
                'target': st.get('target'), 'moving': st.get('moving'),
                'at_limit': st.get('at_limit'), 'held_reason': st.get('held_reason'),
                'commands_sent': st.get('commands_sent'), 'names': st.get('names'),
                'follow': bool(j.get('follow')), 'following': bool(st.get('following')),
                'follow_on': bool(((read_json(j['dir'] / 'control.json') or {}).get('follow')
                                   or {}).get('on')) if j.get('follow') else False,
                'limited': st.get('limited') or []}

    def _kill_later(self, proc, wait=0.5):
        time.sleep(wait)
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass

    # ---- emergency stop ------------------------------------------------------------------------
    def estop(self, source='page'):
        """Stops everything RawBot Studio commands, independent of any page: jog and rollout
        workers get a stop and are killed 0.5 s later; a running calibration is cancelled (its
        original values are written back). Motion that bridge.py itself produces (teleoperation)
        is not under RawBot Studio's control - use the arm's power switch for that."""
        done = []
        if self.jog and self.jog['proc'].poll() is None:
            atomic_write_json(self.jog['dir'] / 'control.json', {'command': 'estop',
                                                                 't': time.time()})
            threading.Thread(target=self._kill_later, args=(self.jog['proc'],),
                             daemon=True).start()
            done.append('jog')
        if self.rollouts:
            act = self.rollouts.active()
            if act:
                self.rollouts.stop_all()
                done.append('rollout %s' % act['id'])
        if self.calib and self.calib['proc'].poll() is None:
            ctl = read_json(self.calib['dir'] / 'control.json') or {}
            ctl.update(command='cancel', t=time.time())
            atomic_write_json(self.calib['dir'] / 'control.json', ctl)
            done.append('calibration')
        if self.sim.running and self.sim.drive != 'manual':
            try:
                self.sim.command({'command': 'stop'})
                done.append('simulated robot')
            except ValueError:
                pass
        rt = self.active_robot
        self._event(rt, 'estop', 'Emergency stop (%s): %s.' % (
            source, ', '.join(done) if done else 'nothing was moving'), 'err')
        return {'ok': True, 'stopped': done}

    # ---- leader capture -------------------------------------------------------------------------
    def save_capture(self, robot_type, body):
        role = body.get('role') or 'leader'
        names = body.get('names') or []
        samples = body.get('samples') or []
        try:
            samples = [[float(s[0])] + [float(v) for v in s[1]] for s in samples]
        except (TypeError, ValueError, IndexError):
            raise HardwareError('Samples must be [time, [values]] pairs.')
        if any(len(s) != len(names) + 1 for s in samples):
            raise HardwareError('Every sample must have one value per joint.')
        return self.reg.save_capture(robot_type, role, names, samples,
                                     body.get('source') or 'ros')

    def stop(self):
        self._stop = True
        self.sim.stop()
        self.relay.stop()
        for k in list(self.monitors):
            self._stop_monitor(*k, wait=1.0)


def explain_serial_error(text, device):
    """A serial / motor-bus error with its likely cause, in the operator's words."""
    t = str(text or '')
    low = t.lower()
    if 'errno 16' in low or 'resource busy' in low or 'could not exclusively lock' in low:
        why = ('another program has %s open. RawBot Studio cannot see programs of other users: '
               'on Ubuntu this is often ModemManager (sudo systemctl stop ModemManager) or '
               'brltty, or a bridge running in a container. ./scripts/hw-doctor.sh shows it.'
               % device)
    elif 'errno 13' in low or 'permission denied' in low:
        why = ('no permission to open %s: sudo usermod -aG dialout $USER, then log out and in.'
               % device)
    elif 'errno 2' in low or 'no such file' in low or 'errno 19' in low:
        why = '%s disappeared (cable or USB power): re-plug it and Detect again.' % device
    elif 'no status packet' in low or 'incorrect status packet' in low:
        why = ('the board answers but the motors do not: switch on the arm\'s own power supply '
               '(USB powers only the board) and check the 3-pin cable to the first motor.')
    elif 'no module named' in low and 'lerobot' in low:
        why = 'LeRobot is not installed in the selected Python environment.'
    else:
        return t
    return '%s - %s' % (why[0].upper() + why[1:], t.strip().splitlines()[-1][:160] if t.strip()
                        else '')


TICKS_PER_TURN = 4096
TICKS_CENTER = 2048


def ticks_to_rad(ticks, m=None):
    m = m or {}
    v = (float(ticks) - TICKS_CENTER) * 2 * math.pi / TICKS_PER_TURN
    return v * (-1.0 if m.get('sign') == -1 else 1.0) + float(m.get('offset') or 0.0)


def clean_joint_map(jm):
    out = {}
    for name, m in (jm or {}).items():
        if not isinstance(m, dict):
            continue
        try:
            off = float(m.get('offset') or 0.0)
        except (TypeError, ValueError):
            raise HardwareError('%s: the offset must be a number (radians).' % name)
        if abs(off) > 2 * math.pi:
            raise HardwareError('%s: the offset must be within one turn.' % name)
        out[str(name)[:60]] = {'sign': -1 if m.get('sign') == -1 else 1, 'offset': off}
    return out


def _rollout_view(r):
    if not r:
        return None
    live = r.get('live') or {}
    return {'id': r.get('id'), 'status': r.get('status'), 'operator': bool(live.get('operator')),
            'interventions': live.get('interventions') or [], 'op_since': live.get('op_since'),
            'workspace': live.get('workspace'), 'step': live.get('step'),
            'pause_reason': live.get('pause_reason')}


def _bridge(url):
    from physios_trainer.rollout_worker import Bridge
    return Bridge(url)


def _token(cfg):
    import hashlib
    return hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:24]


def _port_view(p):
    if not p:
        return None
    return {k: p.get(k) for k in ('device', 'by_id', 'vid', 'pid', 'serial', 'manufacturer',
                                  'product', 'usb_path', 'unique', 'access', 'own_session',
                                  'override')}


def _probe_view(pr):
    if not pr:
        return None
    return {k: pr.get(k) for k in ('ok', 'error', 'found', 'missing_ids', 'unexpected_ids',
                                   'calibration_state', 'checked_at', 'baudrate', 'skipped')}
