# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""The simulated robot: a kinematic virtual arm that runs in the host service.

It has the joints of the robot configuration and follows a target at a limited joint speed
(like a position servo). The target comes from one drive at a time:

    manual   - GUI sliders (absolute targets) or keyboard / gamepad (held velocities that
               expire after 0.3 s, so a released key or a closed page stops the motion)
    leader   - the leader arm's joints (ROS leader topic): the leader drives the simulation
    follower - the physical follower's measured joints: physical-to-simulation mirror
    script   - a list of waypoints, played in order (optionally looping)
    replay   - a recorded trajectory (timestamps + joint rows), played back in time

It always publishes sensor_msgs/JointState (positions and velocities) on SIM_TOPIC. It also
publishes the follower state topic of the robot configuration, but only when the manager
allows it (the configuration has no physical follower and nothing else publishes that
topic): physical_ai_server then records the simulated robot exactly like a follower arm, so
the dataset format does not change. It never publishes on the command (leader) topic, so it
cannot move a physical arm.
"""

import math
import threading
import time

SIM_TOPIC = '/rawbot/sim/joint_states'
# the leader arm's joints when RawBot Studio reads the leader itself (serial session, no
# bridge.py): a JointState that physical_ai_server records as the action
# (action_source 'leader_state'). Not a command topic: nothing moves an arm from it.
LEADER_STATE_TOPIC = '/rawbot/leader/joint_states'
LEADER_STALE_S = 0.5    # older leader readings: the simulated follower holds
RATE_HZ = 30.0
HOLD_S = 0.3            # a keyboard / gamepad velocity expires after this
DEFAULT_SPEED = 1.5     # joint units per second (rad/s for the SO-101 bridge)
SO101_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll',
                'gripper']
DRIVES = ('manual', 'leader', 'follower', 'script', 'replay')


def _now():
    return time.time()


def joint_state(names, pos, vel):
    t = _now()
    return {'header': {'stamp': {'sec': int(t), 'nanosec': int((t % 1) * 1e9)},
                       'frame_id': 'rawbot_sim'},
            'name': list(names), 'position': [float(p) for p in pos],
            'velocity': [float(v) for v in vel], 'effort': []}


class SimRobot:
    """``bridge_factory(url)`` returns an object with advertise/publish/alive (rollout_worker
    Bridge); ``sampler(topic)`` returns (age, names, values) like RosWatch.sample."""

    def __init__(self, url, sampler, bridge_factory=None, log=print, thread=True):
        self.url = url
        self.sampler = sampler
        self.bridge_factory = bridge_factory
        self.log = log
        self.lock = threading.RLock()
        self.robot_type = None
        self.running = False
        self.names = list(SO101_JOINTS)
        self.lo = [-math.pi] * 6
        self.hi = [math.pi] * 6
        self.speed = DEFAULT_SPEED
        self.pos = [0.0] * 6
        self.vel = [0.0] * 6
        self.target = [0.0] * 6
        self.drive = 'manual'
        self.hold = {}            # joint index -> (v in -1..1, t)
        self.script = None        # {points: [[...]], i, loop, hold_s, reached_at}
        self.replay = None        # {t: [...], rows: [[...]], t0, speed}
        self.topics = {}          # role -> topic (from the hardware setup)
        self.state_topic = None   # follower topic the sim may publish (set by the manager)
        self.br = None
        self.error = None
        self.published = 0
        self.started_at = None
        self.rec = None           # trajectory capture: {'t0': float, 'samples': [[t, ...]]}
        # leader input: () -> (age, names, values, extra) when RawBot Studio reads the leader
        # over serial; otherwise the leader ROS topic is sampled
        self.leader_sampler = None
        self.leader_pub = False   # publish the leader joints on LEADER_STATE_TOPIC
        self.leader_in = None     # last leader reading as used: names, values, age, ...
        self._last = None
        self._stop = False
        if thread:
            threading.Thread(target=self._loop, daemon=True).start()

    # ---- configuration ----------------------------------------------------------------
    def ensure(self, robot_type, run, topics, names=None, state_topic=None,
               leader_sampler=None, leader_pub=False, limits=None):
        """Called by the manager on every status: start / stop, joints, allowed topics.
        limits: the robot's joint limits (URDF), applied when the joints change."""
        with self.lock:
            self.default_limits = dict(limits or {})
            self.leader_sampler = leader_sampler if run else None
            self.leader_pub = bool(run and leader_pub and leader_sampler)
            if robot_type != self.robot_type:
                self.robot_type = robot_type
                self._reset(names or SO101_JOINTS)
            elif names and list(names) != self.names:
                self._reset(names)
            self.topics = dict(topics or {})
            self.state_topic = state_topic if run else None
            if run and not self.running:
                self.running = True
                self.started_at = _now()
                self._last = None
            elif not run and self.running:
                self.running = False
                self.drive = 'manual'
                self.hold = {}
                self.script = self.replay = None
                self.rec = None

    def _reset(self, names):
        n = len(names)
        self.names = list(names)
        dl = getattr(self, 'default_limits', None) or {}
        self.lo = [float(dl[x][0]) if x in dl else -math.pi for x in names]
        self.hi = [float(dl[x][1]) if x in dl else math.pi for x in names]
        self.pos = [min(max(0.0, lo), hi) for lo, hi in zip(self.lo, self.hi)]
        self.vel = [0.0] * n
        self.target = list(self.pos)
        self.hold = {}
        self.script = self.replay = None
        self.drive = 'manual'
        self.rec = None

    def _idx(self, name):
        try:
            return self.names.index(name)
        except ValueError:
            raise ValueError('The simulated robot has no joint "%s".' % name)

    # ---- commands ------------------------------------------------------------------------
    def command(self, body):
        cmd = (body or {}).get('command')
        with self.lock:
            if not self.running:
                raise ValueError('The simulated robot is off. Turn on the simulated twin or '
                                 'choose Simulation Only.')
            if cmd == 'drive':
                d = body.get('drive')
                if d not in DRIVES:
                    raise ValueError('Unknown drive "%s".' % d)
                self.drive = d
                self.hold = {}
                if d != 'script':
                    self.script = None
                if d != 'replay':
                    self.replay = None
                if d == 'manual':
                    self.target = list(self.pos)
            elif cmd == 'targets':
                self._manual()
                for k, v in (body.get('targets') or {}).items():
                    i = self._idx(k)
                    self.target[i] = self._clamp(i, float(v))
            elif cmd == 'velocity':
                self._manual()
                t = _now()
                for k, v in (body.get('velocity') or {}).items():
                    i = self._idx(k)
                    v = max(-1.0, min(1.0, float(v)))
                    if v:
                        self.hold[i] = (v, t)
                    else:
                        self.hold.pop(i, None)
            elif cmd == 'release':
                self.hold = {}
                self.target = list(self.pos)
            elif cmd == 'home':
                self._manual()
                self.target = [self._clamp(i, 0.0) for i in range(len(self.names))]
            elif cmd == 'script':
                pts = [self._row(p) for p in body.get('points') or []]
                if not pts:
                    raise ValueError('A script needs at least one waypoint.')
                self.drive = 'script'
                self.replay = None
                self.script = {'points': pts, 'i': 0, 'loop': bool(body.get('loop')),
                               'hold_s': max(0.0, float(body.get('hold_s', 0.5))),
                               'reached_at': None, 'done': False}
            elif cmd == 'replay':
                names = list(body.get('names') or [])
                rows = body.get('rows') or []
                ts = [float(t) for t in body.get('t') or []]
                if not rows or len(rows) != len(ts):
                    raise ValueError('A replay needs matching timestamps and joint rows.')
                cols = [names.index(n) if n in names else None for n in self.names]
                if all(c is None for c in cols):
                    raise ValueError('The trajectory has none of the joints of this robot.')
                mapped = [[(r[c] if c is not None else self.pos[i]) for i, c in
                           enumerate(cols)] for r in rows]
                self.drive = 'replay'
                self.script = None
                self.replay = {'t': [t - ts[0] for t in ts], 'rows': mapped, 'el': 0.0,
                               'speed': max(0.1, min(4.0, float(body.get('speed', 1.0)))),
                               'done': False, 'label': str(body.get('label') or '')[:120]}
            elif cmd == 'stop':
                self.drive = 'manual'
                self.script = self.replay = None
                self.hold = {}
                self.target = list(self.pos)
            elif cmd == 'capture_start':
                self.rec = {'t0': _now(), 'samples': []}
            elif cmd == 'capture_cancel':
                self.rec = None
            elif cmd == 'speed':
                self.speed = max(0.05, min(6.0, float(body.get('speed'))))
            elif cmd == 'limits':
                for k, (lo, hi) in (body.get('limits') or {}).items():
                    i = self._idx(k)
                    lo, hi = float(lo), float(hi)
                    if lo >= hi:
                        raise ValueError('%s: min must be below max.' % k)
                    self.lo[i], self.hi[i] = lo, hi
                    self.target[i] = self._clamp(i, self.target[i])
            else:
                raise ValueError('Unknown simulated-robot command "%s".' % cmd)
            return self.view()

    def read_leader(self):
        """(age, names, values, extra) of the leader arm, from RawBot Studio's serial session
        or the leader ROS topic (bridge.py)."""
        if self.leader_sampler is not None:
            got = self.leader_sampler()
            if got and got[0] is not None:
                return got
        topic = self.topics.get('leader')
        if not topic:
            return None, [], [], {}
        age, names, vals = self.sampler(topic)
        return age, names, vals, {'source': 'ros', 'topic': topic}

    def _follow_leader(self):
        """Joint mapping by name, joint-limit check, then the target. Stale input holds."""
        age, names, vals, extra = self.read_leader()
        info = {'age_s': None if age is None else round(age, 3), 'names': list(names),
                'values': [float(v) for v in vals], 'at_limit': [],
                'missing': [n for n in self.names if n not in names],
                'source': extra.get('source'), 'topic': extra.get('topic'),
                'ticks': extra.get('ticks'), 'rate_hz': extra.get('rate_hz'),
                'stale': age is None or age > LEADER_STALE_S}
        if not info['stale']:
            for i, nme in enumerate(self.names):
                if nme in names:
                    v = float(vals[names.index(nme)])
                    c = self._clamp(i, v)
                    if abs(c - v) > 1e-9:
                        info['at_limit'].append(nme)
                    self.target[i] = c
        self.leader_in = info

    def take_capture(self):
        """End the trajectory capture: (names, [[t, values...]]) or None."""
        with self.lock:
            rec, self.rec = self.rec, None
            if not rec or not rec['samples']:
                return None
            return list(self.names), rec['samples']

    def _manual(self):
        if self.drive != 'manual':
            self.drive = 'manual'
            self.script = self.replay = None
            self.target = list(self.pos)

    def _row(self, p):
        if isinstance(p, dict):
            return [self._clamp(i, float(p.get(n, self.pos[i]))) for i, n in
                    enumerate(self.names)]
        if len(p) != len(self.names):
            raise ValueError('Every waypoint needs one value per joint.')
        return [self._clamp(i, float(v)) for i, v in enumerate(p)]

    def _clamp(self, i, v):
        return max(self.lo[i], min(self.hi[i], v))

    # ---- simulation step -------------------------------------------------------------------
    def step(self, dt):
        with self.lock:
            if not self.running:
                return
            n = len(self.names)
            now = _now()
            if self.drive == 'manual':
                for i, (v, t) in list(self.hold.items()):
                    if now - t > HOLD_S:
                        self.hold.pop(i, None)
                        continue
                    self.target[i] = self._clamp(i, self.target[i] + v * self.speed * dt)
            elif self.drive == 'leader':
                self._follow_leader()
            elif self.drive == 'follower':
                topic = self.topics.get(self.drive)
                age, names, vals = self.sampler(topic) if topic else (None, [], [])
                if age is not None and age < 1.0:
                    for i, nme in enumerate(self.names):
                        if nme in names:
                            self.target[i] = self._clamp(i, float(vals[names.index(nme)]))
            elif self.drive == 'script' and self.script:
                s = self.script
                if not s['done']:
                    goal = s['points'][s['i']]
                    if max(abs(a - b) for a, b in zip(self.pos, goal)) < 1e-3:
                        s['reached_at'] = s['reached_at'] or now
                        if now - s['reached_at'] >= s['hold_s']:
                            s['reached_at'] = None
                            s['i'] += 1
                            if s['i'] >= len(s['points']):
                                if s['loop']:
                                    s['i'] = 0
                                else:
                                    s['i'] = len(s['points']) - 1
                                    s['done'] = True
                    self.target = list(s['points'][s['i']])
            elif self.drive == 'replay' and self.replay:
                r = self.replay
                r['el'] += dt * r['speed']
                el = r['el']
                ts = r['t']
                if el >= ts[-1]:
                    self.target = list(r['rows'][-1])
                    r['done'] = True
                else:
                    k = max(0, next(j for j, t in enumerate(ts) if t > el) - 1)
                    t0, t1 = ts[k], ts[min(k + 1, len(ts) - 1)]
                    a = 0.0 if t1 <= t0 else (el - t0) / (t1 - t0)
                    lo, hi = r['rows'][k], r['rows'][min(k + 1, len(ts) - 1)]
                    self.target = [self._clamp(i, lo[i] + a * (hi[i] - lo[i]))
                                   for i in range(n)]
            # the leader, the mirror and replays follow closely; everything else at the
            # joint speed
            fast = self.drive in ('leader', 'follower', 'replay')
            lim = (self.speed * (4.0 if fast else 1.0)) * dt
            for i in range(n):
                d = self.target[i] - self.pos[i]
                d = max(-lim, min(lim, d))
                self.pos[i] += d
                self.vel[i] = d / dt if dt > 0 else 0.0
            if self.rec is not None and len(self.rec['samples']) < 200000:
                self.rec['samples'].append([round(now - self.rec['t0'], 4)] + [
                    round(p, 5) for p in self.pos])

    # ---- publishing ------------------------------------------------------------------------
    def _publish(self):
        with self.lock:
            if not self.running:
                return
            msg = joint_state(self.names, self.pos, self.vel)
            topics = [SIM_TOPIC] + ([self.state_topic] if self.state_topic else [])
            li = self.leader_in
            lead = None
            if self.leader_pub and self.drive == 'leader' and li and not li['stale']:
                lead = joint_state(li['names'], li['values'], [])
        if self.bridge_factory is None:
            return
        try:
            if self.br is None or not getattr(self.br, 'alive', True):
                self.br = self.bridge_factory(self.url)
                self._advertised = set()
            for t in topics:
                if t not in self._advertised:
                    self.br.advertise(t, 'sensor_msgs/msg/JointState')
                    self._advertised.add(t)
                self.br.publish(t, msg)
            if lead is not None:
                if LEADER_STATE_TOPIC not in self._advertised:
                    self.br.advertise(LEADER_STATE_TOPIC, 'sensor_msgs/msg/JointState')
                    self._advertised.add(LEADER_STATE_TOPIC)
                self.br.publish(LEADER_STATE_TOPIC, lead)
            self.published += 1
            self.error = None
        except Exception as e:  # noqa: BLE001
            self.error = 'Cannot publish over rosbridge (%s): %s' % (self.url, str(e)[:160])
            self.br = None

    def _loop(self):
        period = 1.0 / RATE_HZ
        while not self._stop:
            t = _now()
            if self.running:
                dt = period if self._last is None else min(0.2, t - self._last)
                self._last = t
                self.step(dt)
                self._publish()
                if self.error:
                    time.sleep(1.0)   # rosbridge not reachable: retry slowly
            else:
                self._last = None
            time.sleep(max(0.0, period - (_now() - t)))

    def stop(self):
        self._stop = True

    # ---- view ------------------------------------------------------------------------------
    def owns(self, topic):
        return bool(self.running and topic and topic == self.state_topic)

    def view(self):
        with self.lock:
            out = {'running': self.running, 'drive': self.drive, 'names': list(self.names),
                   'positions': [round(p, 5) for p in self.pos],
                   'velocities': [round(v, 5) for v in self.vel],
                   'target': [round(p, 5) for p in self.target],
                   'limits': [[lo, hi] for lo, hi in zip(self.lo, self.hi)],
                   'speed': self.speed, 'topic': SIM_TOPIC,
                   'state_topic': self.state_topic if self.running else None,
                   'error': self.error if self.running else None,
                   'published': self.published, 'rate_hz': RATE_HZ}
            if self.script:
                out['script'] = {'index': self.script['i'], 'count': len(
                    self.script['points']), 'loop': self.script['loop'],
                    'done': self.script['done']}
            if self.drive == 'leader' and self.leader_in:
                out['leader_in'] = dict(self.leader_in, published=self.leader_pub and
                                        LEADER_STATE_TOPIC, values=[
                                            round(v, 5) for v in self.leader_in['values']])
            if self.rec is not None:
                n = len(self.rec['samples'])
                out['capture'] = {'samples': n, 'duration_s': round(
                    self.rec['samples'][-1][0], 2) if n else 0.0}
            if self.replay:
                r = self.replay
                out['replay'] = {'duration_s': r['t'][-1] / r['speed'], 'elapsed_s': min(
                    r['t'][-1], r['el']) / r['speed'],
                    'done': r['done'], 'label': r['label']}
            return out

    def compare(self, follower_sample):
        """Per-joint simulated minus physical, when the physical follower is live."""
        age, names, vals = follower_sample
        if age is None or age > 1.0 or not names:
            return None
        with self.lock:
            rows = []
            for i, n in enumerate(self.names):
                if n in names:
                    real = float(vals[names.index(n)])
                    rows.append({'name': n, 'sim': round(self.pos[i], 4),
                                 'real': round(real, 4),
                                 'delta': round(self.pos[i] - real, 4)})
        if not rows:
            return None
        worst = max(rows, key=lambda r: abs(r['delta']))
        return {'joints': rows, 'max_abs': abs(worst['delta']), 'worst': worst['name']}
