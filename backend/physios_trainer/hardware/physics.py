# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Simulation backends of the simulated robot.

The simulated robot (simbot.SimRobot) always computes where each joint is told to go (leader
arm, GUI, script, replay). A backend decides what the joints actually do:

    kinematic - built in, always available: the joints reach their targets at a limited
                speed; objects stay where the plan puts them (no physics)
    mujoco    - MuJoCo physics (hardware/mujoco_worker.py, in its own process): the robot from
                its URDF with collision meshes, gravity, contacts, a table and the plan's objects
                as free bodies; the published joints and object poses are the physics result

A backend is offered only when it can run here: MuJoCo needs a Python with the ``mujoco``
package (``PHYSIOS_SIM_PYTHON``, default ``var/sim-venv/bin/python``, made by
``scripts/install.sh --with-mujoco``) and a custom robot with a URDF. Other engines (Gazebo,
Isaac Sim) plug in by adding a class with the same interface as MujocoBackend to BACKENDS -
none is listed until such an adapter exists, because a backend that cannot run is not offered.
"""

import json
import os
from pathlib import Path
import subprocess
import threading
import time

PROBE_TTL_S = 30.0
STATE_STALE_S = 0.5


def default_sim_python(repo):
    env = os.environ.get('PHYSIOS_SIM_PYTHON')
    if env:
        return env
    return str(Path(repo) / 'var' / 'sim-venv' / 'bin' / 'python')


_probe = {}


def probe_mujoco(python):
    """(available, version or reason). Cached: it starts a Python."""
    hit = _probe.get(python)
    if hit and time.time() - hit[0] < PROBE_TTL_S:
        return hit[1]
    if not python or not os.path.isfile(python):
        out = (False, 'No simulation Python at %s. Install it with: '
                      './scripts/install.sh --with-mujoco' % python)
    else:
        try:
            r = subprocess.run([python, '-c', 'import mujoco, sys; sys.stdout.write('
                                'mujoco.__version__)'], capture_output=True, text=True,
                               timeout=20)
            out = (True, r.stdout.strip()) if r.returncode == 0 else (
                False, 'MuJoCo is not installed in %s (pip install mujoco).' % python)
        except (OSError, subprocess.SubprocessError) as e:
            out = (False, 'Cannot start %s: %s' % (python, e))
    _probe[python] = (time.time(), out)
    return out


def backends(python, custom):
    """The backends for this robot, each {id, name, available, detail}. custom: the custom
    robot's simulation inputs (robotcaps.custom_profile()['sim']) or None."""
    out = [{'id': 'kinematic', 'name': 'Kinematic', 'available': True,
            'detail': 'Built in. Joints follow their targets; objects do not move.'}]
    ok, info = probe_mujoco(python)
    if ok and not (custom and custom.get('urdf')):
        ok, info = False, 'MuJoCo needs the robot\'s URDF: add it in Robots -> Robot setup.'
    out.append({'id': 'mujoco', 'name': 'MuJoCo', 'available': ok,
                'detail': ('Physics %s: gravity, contacts, objects.' % info) if ok else info,
                'installed': probe_mujoco(python)[0]})
    return out


class MujocoBackend:
    """The MuJoCo worker process. set_targets() / latest() are cheap and never block; the
    process is (re)started by sync() when the robot or its objects change."""

    id = 'mujoco'

    def __init__(self, python, log=print):
        self.python = python
        self.log = log
        self.proc = None
        self.key = None
        self.items_key = None
        self.loaded = None
        self.state = None
        self.state_t = 0.0
        self.error = None
        self.notes = []
        self.lock = threading.Lock()

    # ---- process ----------------------------------------------------------------------------
    def _start(self):
        env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[2]))
        self.proc = subprocess.Popen([self.python, '-u', '-m',
                                      'physios_trainer.hardware.mujoco_worker'],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, env=env, bufsize=1)
        proc = self.proc
        threading.Thread(target=self._read, args=(proc,), daemon=True).start()
        threading.Thread(target=self._read_err, args=(proc,), daemon=True).start()

    def _read(self, proc):
        for line in proc.stdout:
            try:
                m = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                if proc is not self.proc:
                    return
                ev = m.get('event')
                if ev == 'state':
                    self.state, self.state_t = m, time.time()
                elif ev == 'loaded':
                    self.loaded, self.error, self.notes = m, None, m.get('notes') or []
                elif ev == 'error':
                    self.error = m.get('message')
        with self.lock:
            if proc is self.proc and not self.error:
                self.error = 'The MuJoCo process stopped (exit %s).' % proc.poll()

    def _read_err(self, proc):
        tail = []
        for line in proc.stderr:
            tail = (tail + [line.rstrip()])[-6:]
        if tail and proc.poll() not in (0, None):
            with self.lock:
                if proc is self.proc:
                    self.error = 'MuJoCo: ' + ' | '.join(tail)[-400:]

    def _send(self, obj):
        p = self.proc
        if p is None or p.poll() is not None:
            return False
        try:
            p.stdin.write(json.dumps(obj) + '\n')
            p.stdin.flush()
            return True
        except (OSError, ValueError):
            return False

    def stop(self):
        p, self.proc = self.proc, None
        self.key = self.items_key = self.loaded = self.state = None
        if p is not None and p.poll() is None:
            try:
                p.stdin.write('{"op": "quit"}\n')
                p.stdin.flush()
                p.wait(timeout=2)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                p.kill()

    # ---- interface used by SimRobot ----------------------------------------------------------
    def sync(self, sim, items, targets):
        """sim: {urdf, mesh_map, joints, reach} of the robot; items: the scene objects (metres).
        Starts / reloads the world when the robot or the set of objects changes."""
        key = json.dumps([sim.get('urdf'), sim.get('joints'), round(sim.get('reach') or 0, 3)])
        ids = json.dumps(sorted((str(i['id']), i.get('role'), i.get('size')) for i in items))
        if self.proc is None or self.proc.poll() is not None:
            if self.key == key and self.error:
                return          # it failed for this robot: do not restart it in a loop
            self.stop()
            self._start()
            self.key = None
        if key != self.key or ids != self.items_key:
            self.key, self.items_key = key, ids
            self.places_key = json.dumps(items, sort_keys=True)
            with self.lock:
                self.loaded = self.state = None
                self.error = None
            self._send({'op': 'targets', 'q': targets})
            self._send({'op': 'load', 'urdf': sim['urdf'], 'mesh_map': sim['mesh_map'],
                        'joints': sim['joints'], 'items': items, 'reach': sim.get('reach')})
        else:
            places = json.dumps(items, sort_keys=True)
            if places != getattr(self, 'places_key', None):
                self.places_key = places
                self._send({'op': 'items', 'items': items})

    def place(self, items):
        """Put the objects back where the scene says (reset after a take)."""
        self.places_key = json.dumps(items, sort_keys=True)
        self._send({'op': 'items', 'items': items})

    def set_targets(self, q):
        self._send({'op': 'targets', 'q': q})

    def latest(self):
        with self.lock:
            if self.state is None or time.time() - self.state_t > STATE_STALE_S:
                return None
            return self.state

    def view(self):
        with self.lock:
            st = self.state
            return {'backend': self.id, 'running': bool(self.proc and self.proc.poll() is None),
                    'loaded': bool(self.loaded), 'error': self.error, 'notes': self.notes,
                    'version': (self.loaded or {}).get('mujoco'),
                    'live': bool(st and time.time() - self.state_t < STATE_STALE_S),
                    'track_err': st and st.get('track_err'),
                    'contacts': st and st.get('contacts')}
