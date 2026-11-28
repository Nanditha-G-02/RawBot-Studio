# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""MuJoCo physics for the simulated robot - runs as its own process, in a Python that has the
``mujoco`` package (the host service itself stays standard-library only).

The world is built from the robot's own URDF (the uploaded custom robot): its links, collision
meshes, joint axes and limits, plus a table sized to the robot's reach, the plan's objects
(free bodies) and targets (trays), a floor, gravity and contacts. Every movable joint gets a
stiff position actuator (gravity compensated, like a servo arm) whose target is the joint
value the simulated robot is told to reach (leader arm, GUI, script, replay); the published
joint state is what the physics produced, so a joint blocked by the table or an object shows
it. Grasping works through contacts when the URDF has gripper joints.

Protocol: JSON lines. stdin:
    {"op": "load", "urdf": path, "mesh_map": {filename: abs path}, "joints": [urdf names],
     "items": [{"id", "role", "x", "y", "yaw", "size"}], "reach": metres}
    {"op": "targets", "q": {urdf joint: value}}
    {"op": "items", "items": [...]}          re-place the objects (new episode / reset)
    {"op": "quit"}
stdout:
    {"event": "loaded", "joints": [...], "bodies": n, "notes": [...], "mujoco": version}
    {"event": "state", "t": s, "q": {joint: value}, "v": {...}, "objects": [{"id", "p", "q"}],
     "track_err": max |target - q|, "contacts": n}      at 30 Hz
    {"event": "error", "message": ...}
Frames: the robot base frame (URDF, Z-up, metres); the table top is z = 0.
"""

import json
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET

PUBLISH_HZ = 30.0
TIMESTEP = 0.002
TABLE_H = 0.74
KP_REVOLUTE, ARM_REVOLUTE = 10000.0, 0.1     # max tracking error ~0.005 rad on a 1 rad/s sweep
KP_PRISMATIC, ARM_PRISMATIC = 4000.0, 0.05
SUPPORTED_MESH = ('.stl', '.obj', '.msh')
ROBOT_CONTYPE, ROBOT_CONAFFINITY = 2, 1     # no robot-robot contacts (see World.load)


def emit(obj):
    sys.stdout.write(json.dumps(obj) + '\n')
    sys.stdout.flush()


def prepare_urdf(text, mesh_map):
    """URDF text MuJoCo can compile: mesh paths made absolute, meshes MuJoCo cannot read (DAE)
    swapped for a sibling STL/OBJ when the package has one, links without collision geometry
    given their visual geometry for contacts. Returns (text, notes, mimics)."""
    notes = []
    root = ET.fromstring(text)
    for tag in ('gazebo', 'transmission', 'ros2_control'):
        for el in root.findall(tag):
            root.remove(el)
    by_stem = {}
    for k, p in mesh_map.items():
        stem, ext = os.path.splitext(p)
        if ext.lower() in SUPPORTED_MESH:
            by_stem.setdefault(os.path.basename(stem).lower(), p)

    def fix(geom_parent, where):
        for mesh in geom_parent.iter('mesh'):
            fn = mesh.get('filename', '')
            path = mesh_map.get(fn)
            if path and os.path.splitext(path)[1].lower() not in SUPPORTED_MESH:
                stem = os.path.splitext(path)[0]
                # a sibling with the same name in the same folder, else anywhere in the package
                alt = next((stem + e for e in SUPPORTED_MESH + tuple(x.upper() for x in
                                                                     SUPPORTED_MESH)
                            if os.path.isfile(stem + e)), None)
                path = alt or by_stem.get(os.path.basename(stem).lower())
            if not path or not os.path.isfile(path):
                return False
            mesh.set('filename', path)
        return True

    for link in root.findall('link'):
        cols = link.findall('collision')
        for c in cols:
            if not fix(c, 'collision'):
                link.remove(c)
                notes.append('%s: a collision mesh MuJoCo cannot read was left out'
                             % link.get('name'))
        if not link.findall('collision'):
            for v in link.findall('visual'):
                c = ET.SubElement(link, 'collision')
                for child in list(v):
                    if child.tag in ('origin', 'geometry'):
                        c.append(ET.fromstring(ET.tostring(child)))
                if not fix(c, 'collision'):
                    link.remove(c)
        for v in link.findall('visual'):
            link.remove(v)      # MuJoCo discards visuals; RawBot Studio draws them in three.js
    mimics = []
    for j in root.findall('joint'):
        m = j.find('mimic')
        if m is not None:
            mimics.append((j.get('name'), m.get('joint'), float(m.get('multiplier', 1)),
                           float(m.get('offset', 0))))
            j.remove(m)
    return ET.tostring(root, encoding='unicode'), notes, mimics


class World:
    def __init__(self, mujoco):
        self.mj = mujoco
        self.m = self.d = None
        self.lock = threading.Lock()
        self.targets = {}
        self.mimic = {}
        self.mimic_act = {}
        self.items = []

    def load(self, req):
        mj = self.mj
        with open(req['urdf'], encoding='utf-8') as f:
            text, notes, mimics = prepare_urdf(f.read(), req.get('mesh_map') or {})
        robot = mj.MjSpec.from_string(text)
        w = mj.MjSpec()
        w.option.timestep = TIMESTEP
        w.option.integrator = mj.mjtIntegrator.mjINT_IMPLICITFAST
        w.option.cone = mj.mjtCone.mjCONE_ELLIPTIC
        reach = max(0.15, float(req.get('reach') or 0.4))
        sx, sy = max(0.6, reach * 2.3), max(0.6, reach * 2.4)
        cx = sx / 2 - max(0.12, reach * 0.35)
        wb = w.worldbody
        floor = wb.add_geom(name='floor')
        floor.type = mj.mjtGeom.mjGEOM_PLANE
        floor.size = [6, 6, 0.1]
        floor.pos = [0, 0, -TABLE_H]
        table = wb.add_geom(name='table')
        table.type = mj.mjtGeom.mjGEOM_BOX
        table.size = [sx / 2, sy / 2, 0.015]
        table.pos = [cx, 0, -0.015]
        table.friction = [0.9, 0.005, 0.0001]
        w.attach(robot, frame=wb.add_frame(), prefix='robot/')
        want = set(req.get('joints') or [])
        self.joints = []
        for j in w.joints:
            name = j.name[len('robot/'):]
            if want and name not in want:
                continue
            prism = j.type == mj.mjtJoint.mjJNT_SLIDE
            kp, arm = (KP_PRISMATIC, ARM_PRISMATIC) if prism else (KP_REVOLUTE, ARM_REVOLUTE)
            j.armature = arm
            j.frictionloss = 0.0
            j.damping = [0.1, 0, 0]
            j.actfrclimited = 0      # URDF effort limits are often placeholders: follow the arm
            a = w.add_actuator(name='act_' + name)
            a.target = j.name
            a.trntype = mj.mjtTrn.mjTRN_JOINT
            a.set_to_position(kp=kp, kv=2 * math.sqrt(kp * arm) + 1)
            self.joints.append(name)
        self.mimic = {}
        for name, leader, mult, off in mimics:
            # the follower joint is driven like its leader (both fingers of a gripper squeeze
            # with the same force, so a cube is centred); the equality keeps them in step
            if leader in self.joints and name not in self.joints:
                jm = next((j for j in w.joints if j.name == 'robot/' + name), None)
                if jm is not None:
                    prism = jm.type == mj.mjtJoint.mjJNT_SLIDE
                    kp, arm = (KP_PRISMATIC, ARM_PRISMATIC) if prism else (KP_REVOLUTE,
                                                                           ARM_REVOLUTE)
                    jm.armature = arm
                    jm.frictionloss = 0.0
                    jm.damping = [0.1, 0, 0]
                    am = w.add_actuator(name='act_' + name)
                    am.target = jm.name
                    am.trntype = mj.mjtTrn.mjTRN_JOINT
                    am.set_to_position(kp=kp, kv=2 * math.sqrt(kp * arm) + 1)
                    self.mimic[name] = (leader, mult, off)
            e = w.add_equality()
            e.type = mj.mjtEq.mjEQ_JOINT
            e.name1, e.name2 = 'robot/' + name, 'robot/' + leader
            e.data[0:5] = [off, mult, 0, 0, 0]
        # the robot touches the table and the objects, never itself: the pose is the leader's
        # (a physical arm), and URDF collision meshes of neighbouring links usually overlap.
        # Robot geoms: contype 2 / conaffinity 1; world and objects: 1 / 1 (the default).
        for b in w.bodies:
            if b.name.startswith('robot/'):
                b.gravcomp = 1.0
                for g in b.geoms:
                    g.friction = [1.2, 0.01, 0.0002]
                    g.condim = 4
                    g.contype, g.conaffinity = ROBOT_CONTYPE, ROBOT_CONAFFINITY
        self.item_bodies = []
        for it in req.get('items') or []:
            self._add_item(w, it)
        self.m = w.compile()
        self.d = mj.MjData(self.m)
        self.qadr = {n: self.m.jnt_qposadr[self.m.joint('robot/' + n).id] for n in self.joints}
        self.vadr = {n: self.m.jnt_dofadr[self.m.joint('robot/' + n).id] for n in self.joints}
        self.act = {n: self.m.actuator('act_' + n).id for n in self.joints}
        self.mimic_act = {n: self.m.actuator('act_' + n).id for n in self.mimic}
        self.rng = {n: tuple(self.m.jnt_range[self.m.joint('robot/' + n).id]) for n in
                    self.joints}
        self.limited = {n: bool(self.m.jnt_limited[self.m.joint('robot/' + n).id]) for n in
                        self.joints}
        self.items = list(req.get('items') or [])
        self._place_items(self.items)
        # start where the robot is told to be, not in a collapse from zero
        for n, v in self.targets.items():
            if n in self.qadr:
                self.d.qpos[self.qadr[n]] = self._clip(n, v)
        mj.mj_forward(self.m, self.d)
        return {'event': 'loaded', 'joints': self.joints, 'bodies': int(self.m.nbody),
                'notes': notes, 'mujoco': getattr(mj, '__version__', '?'),
                'items': [b[0] for b in self.item_bodies]}

    def _add_item(self, w, it):
        mj = self.mj
        s = max(0.01, float(it.get('size') or 0.03))
        b = w.worldbody.add_body(name='item_' + str(it['id']))
        b.add_freejoint()
        if it.get('role') == 'target':
            wall, h = 0.006, min(0.04, s * 0.25)
            parts = [([s / 2, s / 2, wall / 2], [0, 0, wall / 2])]
            for a, c in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                size = [wall / 2 if a else s / 2, wall / 2 if c else s / 2, h / 2]
                parts.append((size, [a * (s - wall) / 2, c * (s - wall) / 2, h / 2]))
            for size, pos in parts:
                g = b.add_geom()
                g.type = mj.mjtGeom.mjGEOM_BOX
                g.size, g.pos = size, pos
                g.mass = 0.25 / len(parts)
                g.friction = [1.0, 0.005, 0.0001]
        else:
            g = b.add_geom()
            shape = it.get('shape')              # a cube unless the item says otherwise
            if shape == 'cylinder':
                g.type, g.size = mj.mjtGeom.mjGEOM_CYLINDER, [s / 2, s / 2, 0]
            elif shape == 'sphere':
                g.type, g.size = mj.mjtGeom.mjGEOM_SPHERE, [s / 2, 0, 0]
            else:
                g.type, g.size = mj.mjtGeom.mjGEOM_BOX, [s / 2] * 3
            g.pos = [0, 0, s / 2]
            g.mass = 0.05 * (s / 0.03) ** 3
            g.friction = [1.0, 0.01, 0.0002]
            g.condim = 4
        self.item_bodies.append((str(it['id']), 'item_' + str(it['id'])))

    def _place_items(self, items):
        by_id = {str(i['id']): i for i in items}
        for iid, body in self.item_bodies:
            it = by_id.get(iid)
            if not it:
                continue
            j = self.m.body(body).jntadr[0]
            a = self.m.jnt_qposadr[j]
            yaw = math.radians(float(it.get('yaw') or 0))
            self.d.qpos[a:a + 7] = [float(it['x']), float(it['y']), 0.0005, math.cos(yaw / 2), 0,
                                    0, math.sin(yaw / 2)]
            da = self.m.jnt_dofadr[j]
            self.d.qvel[da:da + 6] = 0

    def _clip(self, n, v):
        if self.limited.get(n):
            lo, hi = self.rng[n]
            return min(max(v, lo), hi)
        return v

    def set_targets(self, q):
        with self.lock:
            self.targets.update({k: float(v) for k, v in q.items()})

    def step_for(self, seconds):
        mj = self.mj
        with self.lock:
            for n, v in self.targets.items():
                if n in self.act:
                    self.d.ctrl[self.act[n]] = self._clip(n, v)
            for n, (leader, mult, off) in self.mimic.items():
                if leader in self.targets:
                    self.d.ctrl[self.mimic_act[n]] = off + mult * self._clip(
                        leader, self.targets[leader])
        steps = max(1, int(round(seconds / TIMESTEP)))
        for _ in range(steps):
            mj.mj_step(self.m, self.d)

    def state(self):
        q = {n: float(self.d.qpos[a]) for n, a in self.qadr.items()}
        v = {n: float(self.d.qvel[a]) for n, a in self.vadr.items()}
        objs = []
        for iid, body in self.item_bodies:
            bid = self.m.body(body).id
            p = self.d.xpos[bid]
            qq = self.d.xquat[bid]
            objs.append({'id': iid, 'p': [round(float(x), 5) for x in p],
                         'q': [round(float(x), 5) for x in qq]})
        err = 0.0
        with self.lock:
            for n, t in self.targets.items():
                if n in q:
                    err = max(err, abs(self._clip(n, t) - q[n]))
        return {'event': 'state', 't': round(float(self.d.time), 4),
                'q': {k: round(x, 6) for k, x in q.items()},
                'v': {k: round(x, 5) for k, x in v.items()}, 'objects': objs,
                'track_err': round(err, 5), 'contacts': int(self.d.ncon)}


def main():
    try:
        import mujoco
    except ImportError as e:
        emit({'event': 'error', 'message': 'MuJoCo is not installed in this Python: %s' % e})
        return 2
    world = World(mujoco)
    inbox = []
    cv = threading.Condition()

    def reader():
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            with cv:
                inbox.append(msg)
                cv.notify()
        with cv:
            inbox.append({'op': 'quit'})
            cv.notify()

    threading.Thread(target=reader, daemon=True).start()
    period = 1.0 / PUBLISH_HZ
    nxt = time.monotonic()
    while True:
        with cv:
            if world.m is None and not inbox:
                cv.wait(0.5)
            msgs, inbox[:] = list(inbox), []
        for msg in msgs:
            op = msg.get('op')
            try:
                if op == 'quit':
                    return 0
                if op == 'load':
                    emit(world.load(msg))
                    nxt = time.monotonic()
                elif op == 'targets':
                    world.set_targets(msg.get('q') or {})
                elif op == 'items' and world.m is not None:
                    world.items = list(msg.get('items') or [])
                    world._place_items(world.items)
            except Exception as e:  # noqa: BLE001 - reported to the service, never fatal
                emit({'event': 'error', 'message': '%s: %s' % (op, str(e)[:400])})
        if world.m is None:
            continue
        world.step_for(period)
        emit(world.state())
        nxt += period
        time.sleep(max(0.0, nxt - time.monotonic()))
        if time.monotonic() - nxt > 1.0:
            nxt = time.monotonic()      # fell behind (slow machine): do not try to catch up


if __name__ == '__main__':
    sys.exit(main())
