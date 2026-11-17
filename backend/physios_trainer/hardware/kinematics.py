# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Robot-independent kinematics and workspace limits (stdlib only, so the motion workers can
load it in their own process).

A robot is a serial chain built from URDF joints (``robots/urdf.py`` summary format: name,
type, parent, child, xyz, rpy, axis, lower, upper). The chain runs from the root link to the
deepest link; the tool point is that link's origin. Forward kinematics gives the tool position
in the base frame; inverse kinematics moves it by damped least squares on position (any arm,
no closed form). The gripper and joints that do not move the tool are left alone.

Workspace limits (table frame, millimetres, base at the origin - planning/profiles.py):
  * the tool never goes below the table (``floor_mm``, default 0)
  * near the table (below ``table_band_mm``) it stays over the table area (x/y limits + margin)
  * low around the base (below ``base_band_mm``) it keeps ``base_keepout_mm`` away
  * keep-out boxes from the workspace are columns it may not enter below ``keepout_h_mm``

These are checks on the commanded pose. The SO-101 model is the approximate chain of the 3D
view, so they are guards with a margin, not collision detection.
"""

import math

GRIPPER_HINTS = ('grip', 'finger', 'jaw', 'claw', 'hand', 'pinch')

# SO-101 family (no URDF in RawBot Studio): the same approximate chain as the 3D view
SO101_CHAIN = [
    {'name': 'shoulder_pan', 'type': 'revolute', 'parent': 'base', 'child': 'l1',
     'xyz': [0, 0, 0.05], 'rpy': [0, 0, 0], 'axis': [0, 0, 1]},
    {'name': 'shoulder_lift', 'type': 'revolute', 'parent': 'l1', 'child': 'l2',
     'xyz': [0, 0, 0.065], 'rpy': [0, 0, 0], 'axis': [0, 1, 0]},
    {'name': 'elbow_flex', 'type': 'revolute', 'parent': 'l2', 'child': 'l3',
     'xyz': [0, 0, 0.116], 'rpy': [0, 0, 0], 'axis': [0, 1, 0]},
    {'name': 'wrist_flex', 'type': 'revolute', 'parent': 'l3', 'child': 'l4',
     'xyz': [0.135, 0, 0], 'rpy': [0, 0, 0], 'axis': [0, 1, 0]},
    {'name': 'wrist_roll', 'type': 'revolute', 'parent': 'l4', 'child': 'l5',
     'xyz': [0.05, 0, 0], 'rpy': [0, 0, 0], 'axis': [1, 0, 0]},
    {'name': 'tool', 'type': 'fixed', 'parent': 'l5', 'child': 'tool',
     'xyz': [0.09, 0, 0], 'rpy': [0, 0, 0], 'axis': [1, 0, 0]},
]


# ---- 4x4 transforms -----------------------------------------------------------------------
def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _trans(xyz):
    return [[1, 0, 0, xyz[0]], [0, 1, 0, xyz[1]], [0, 0, 1, xyz[2]], [0, 0, 0, 1]]


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), \
        math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0],
            [-sp, cp * sr, cp * cr, 0], [0, 0, 0, 1]]


def _axis_angle(ax, q):
    n = math.sqrt(sum(a * a for a in ax)) or 1.0
    x, y, z = (a / n for a in ax)
    c, s, t = math.cos(q), math.sin(q), 1 - math.cos(q)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0], [0, 0, 0, 1]]


class Chain:
    """A serial chain from URDF joints; ``names`` maps chain joints to the robot's joint
    names (URDF joint -> ROS joint) when they differ."""

    def __init__(self, joints, names=None):
        names = names or {}
        by_parent = {}
        children = set()
        for j in joints or []:
            if j.get('parent') and j.get('child'):
                by_parent.setdefault(j['parent'], []).append(j)
                children.add(j['child'])
        roots = [p for p in by_parent if p not in children]

        def longest(link, seen=()):
            best = []
            for j in by_parent.get(link, []):
                if j['child'] in seen:
                    continue
                path = [j] + longest(j['child'], seen + (link,))
                moving = sum(1 for x in path if x.get('type') != 'fixed')
                if moving > sum(1 for x in best if x.get('type') != 'fixed') or (
                        moving == sum(1 for x in best if x.get('type') != 'fixed') and
                        len(path) > len(best)):
                    best = path
            return best
        self.chain = max((longest(r) for r in roots), key=lambda p: (
            sum(1 for x in p if x.get('type') != 'fixed'), len(p)), default=[])
        self.joint_of = {j['name']: names.get(j['name'], j['name']) for j in self.chain}
        self.moving = [self.joint_of[j['name']] for j in self.chain if j.get('type') in (
            'revolute', 'continuous', 'prismatic')]
        self.limits = {self.joint_of[j['name']]: (j.get('lower'), j.get('upper'))
                       for j in self.chain if j.get('type') == 'revolute'
                       and isinstance(j.get('lower'), (int, float))
                       and isinstance(j.get('upper'), (int, float))}

    def __bool__(self):
        return bool(self.moving)

    def fk(self, q):
        """Tool position [x, y, z] in metres, base frame. ``q``: joint name -> value."""
        T = _trans([0, 0, 0])
        for j in self.chain:
            T = _mul(T, _mul(_trans(j.get('xyz') or [0, 0, 0]), _rpy(*(j.get('rpy') or
                                                                         [0, 0, 0]))))
            v = float(q.get(self.joint_of[j['name']], 0.0) or 0.0)
            if j.get('type') in ('revolute', 'continuous'):
                T = _mul(T, _axis_angle(j.get('axis') or [1, 0, 0], v))
            elif j.get('type') == 'prismatic':
                ax = j.get('axis') or [1, 0, 0]
                T = _mul(T, _trans([a * v for a in ax]))
        return [T[0][3], T[1][3], T[2][3]]

    def ik(self, q, target, iters=60, tol=1e-4, damping=0.02):
        """Joint values that put the tool at ``target`` (metres), starting from ``q``. The
        gripper is not used. Returns (q_new, error_m)."""
        q = dict(q)
        use = [n for n in self.moving if not any(h in n.lower() for h in GRIPPER_HINTS)]
        err = None
        for _ in range(iters):
            p = self.fk(q)
            e = [target[i] - p[i] for i in range(3)]
            err = math.sqrt(sum(x * x for x in e))
            if err < tol:
                break
            J = []
            for n in use:            # numeric Jacobian, one column per joint
                qd = dict(q)
                qd[n] = float(q.get(n, 0.0)) + 1e-5
                pd = self.fk(qd)
                J.append([(pd[i] - p[i]) / 1e-5 for i in range(3)])
            # dq = J^T (J J^T + l^2 I)^-1 e
            A = [[sum(J[k][i] * J[k][j] for k in range(len(use))) + (damping ** 2 if i == j
                                                                     else 0) for j in range(3)]
                 for i in range(3)]
            y = _solve3(A, e)
            if y is None:
                break
            for k, n in enumerate(use):
                v = float(q.get(n, 0.0)) + sum(J[k][i] * y[i] for i in range(3))
                lo, hi = self.limits.get(n, (None, None))
                if lo is not None:
                    v = max(lo, min(hi, v))
                q[n] = v
        p = self.fk(q)
        return q, math.sqrt(sum((target[i] - p[i]) ** 2 for i in range(3)))


def _solve3(A, b):
    m = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(3):
        piv = max(range(c, 3), key=lambda r: abs(m[r][c]))
        if abs(m[piv][c]) < 1e-12:
            return None
        m[c], m[piv] = m[piv], m[c]
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [m[r][k] - f * m[c][k] for k in range(4)]
    return [m[i][3] / m[i][i] for i in range(3)]


# ---- workspace -------------------------------------------------------------------------------
def workspace_from_profile(ws):
    """Limits from a planning workspace (planning/profiles.default_workspace)."""
    ws = ws or {}
    return {'floor_mm': float(ws.get('floor_mm', 0)),
            'x_min_mm': ws.get('x_min_mm'), 'x_max_mm': ws.get('x_max_mm'),
            'y_min_mm': ws.get('y_min_mm'), 'y_max_mm': ws.get('y_max_mm'),
            'margin_mm': float(ws.get('margin_mm', 10) or 0),
            'base_keepout_mm': float(ws.get('base_keepout_mm', 0) or 0),
            'keepouts': [k for k in ws.get('keepouts') or [] if isinstance(k, dict)],
            'table_band_mm': 40.0, 'base_band_mm': 80.0, 'keepout_h_mm': 100.0}


def check(ws, p):
    """None when the tool position ``p`` (metres) is allowed, else a short reason."""
    if not ws or p is None:
        return None
    x, y, z = (v * 1000 for v in p)
    m = ws.get('margin_mm', 0)
    if z < ws.get('floor_mm', 0):
        return 'below the table (z %.0f mm)' % z
    if z < ws.get('base_band_mm', 80) and math.hypot(x, y) < ws.get('base_keepout_mm', 0):
        return 'too close to the base (%.0f mm)' % math.hypot(x, y)
    if z < ws.get('table_band_mm', 40) and None not in (ws.get('x_min_mm'), ws.get('x_max_mm'),
                                                        ws.get('y_min_mm'), ws.get('y_max_mm')):
        if not (ws['x_min_mm'] - m <= x <= ws['x_max_mm'] + m and
                ws['y_min_mm'] - m <= y <= ws['y_max_mm'] + m):
            return 'outside the table area (x %.0f, y %.0f mm)' % (x, y)
    for k in ws.get('keepouts') or []:
        try:
            inside = (float(k['x_min_mm']) <= x <= float(k['x_max_mm']) and
                      float(k['y_min_mm']) <= y <= float(k['y_max_mm']))
        except (KeyError, TypeError, ValueError):
            continue
        if inside and z < float(k.get('height_mm', ws.get('keepout_h_mm', 100))):
            return 'inside keep-out %s' % (k.get('name') or k.get('id') or '')
    return None


def model_for(robot_type, custom_joints=None, urdf_to_ros=None, has_motor_table=False):
    """The kinematic model of a robot: its URDF (custom robots), the SO-101 chain for the
    SO-101 family (has a Feetech motor table), else None (no workspace checks)."""
    if custom_joints:
        c = Chain(custom_joints, urdf_to_ros)
        return c if c else None
    if has_motor_table:
        return Chain(SO101_CHAIN)
    return None
