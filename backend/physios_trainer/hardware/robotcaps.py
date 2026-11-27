# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""What a robot is, independent of its model: joints, gripper, simulation model, how its
arms can be read and which control methods fit. Planning (planning.py) builds its options
from this, so a new robot needs no planning code.

Sources, in order of preference:
  * a custom robot registered in RawBot Studio (robots/): the URDF's movable joints with their
    type and limits, and the URDF -> ROS joint mapping of its configuration;
  * the robot server's robot configuration (joint_order, sent by the interface);
  * a motor table for RawBot Studio's own serial session (registry.motor_spec), when one exists.

Nothing here names a robot model.
"""

import json
import math
from pathlib import Path
import re
import time

# a joint that opens / closes an end effector, recognised by name (any robot)
GRIPPER_RE = re.compile(r'grip|finger|jaw|claw|hand|pinch|suction|end_?eff', re.I)
KEYBOARD_MAX = 9       # useHeldInput: Q/A ... O/L
GAMEPAD_MAX = 6        # two sticks + bumpers + triggers
_CACHE = {}
_TTL = 5.0


def chain_reach(urdf_path):
    """The longest base-to-tip sum of joint offsets (metres): the robot's reach, whatever its
    zero pose (a folded zero pose makes the model's extent too small)."""
    import xml.etree.ElementTree as ET
    try:
        root = ET.parse(str(urdf_path)).getroot()
    except (OSError, ET.ParseError):
        return 0.0
    kids = {}
    for j in root.findall('joint'):
        par, ch, o = j.find('parent'), j.find('child'), j.find('origin')
        if par is None or ch is None:
            continue
        try:
            xyz = [float(v) for v in ((o.get('xyz') if o is not None else None) or '0 0 0'
                                      ).split()][:3]
        except ValueError:
            xyz = [0.0, 0.0, 0.0]
        kids.setdefault(par.get('link'), []).append((ch.get('link'), math.sqrt(sum(
            v * v for v in xyz))))

    def walk(link, depth=0):
        # the first offset (base height) is not reach
        if depth > 64:
            return 0.0
        return max([(d if depth else 0.0) + walk(c, depth + 1) for c, d in kids.get(link, [])
                    if d > 1e-6 or depth] + [walk(c, depth) for c, d in kids.get(link, [])
                                             if d <= 1e-6 and not depth] or [0.0])

    children = {c for v in kids.values() for c, _ in v}
    return max([walk(r) for r in kids if r not in children] or [0.0])


def custom_profile(robots_dir, robot_type):
    """Joints of a registered custom robot whose robot type is ``robot_type`` (the interface's
    customKey: lower case, non [a-z0-9_] -> '_'), or None."""
    if not robots_dir or not robot_type:
        return None
    hit = _CACHE.get((str(robots_dir), robot_type))
    if hit and time.time() - hit[0] < _TTL:
        return hit[1]
    out = None
    root = Path(robots_dir)
    for d in sorted(root.iterdir()) if root.is_dir() else []:
        try:
            with open(d / 'profile.json') as f:
                p = json.load(f)
        except (OSError, ValueError):
            continue
        key = re.sub(r'[^a-z0-9_]', '_', str(p.get('id') or '').lower())
        if key != robot_type:
            continue
        summary = (p.get('validation') or {}).get('summary') or {}
        urdf = {j['name']: j for j in summary.get('joint_list') or []
                if j.get('type') in ('revolute', 'continuous', 'prismatic')}
        cfg = p.get('config') or {}
        rows = [r for r in cfg.get('joints') or [] if r.get('enabled', True)] or [
            {'urdf_joint': n, 'ros_joint': n} for n in urdf]
        joints = []
        for r in rows:
            u = urdf.get(r.get('urdf_joint')) or {}
            lo = r.get('soft_lower', u.get('lower'))
            hi = r.get('soft_upper', u.get('upper'))
            joints.append({'name': r.get('ros_joint') or r.get('urdf_joint'),
                           'urdf': r.get('urdf_joint'), 'type': u.get('type') or 'revolute',
                           'lower': lo, 'upper': hi})
        # what a physics backend needs to build the robot: its URDF, mesh files, the joints
        # to actuate and its reach, which sizes the table - the same measure as the 3D view
        # (frontend sim/simWorld.js chainReach), so both draw the same table
        files = d / 'files'
        mesh_map = {k: str(files / v) for k, v in (summary.get('mesh_map') or {}).items()}
        sim = None
        if (d / 'resolved.urdf').is_file():
            ext = [float(x) for x in summary.get('extent_m') or [] if isinstance(x, (int, float))]
            reach = chain_reach(d / 'resolved.urdf')
            sim = {'urdf': str(d / 'resolved.urdf'), 'mesh_map': mesh_map,
                   'joints': [r.get('urdf_joint') for r in rows if r.get('urdf_joint') in urdf],
                   'reach': reach if reach > 0.05 else (max(ext) if ext else 0.4)}
        out = {'source': 'custom robot %s' % p.get('name', p.get('id')), 'joints': joints,
               'rows': rows, 'sim': sim,
               'urdf_joints': summary.get('joint_list') or [],
               'urdf_to_ros': {r.get('urdf_joint'): r.get('ros_joint') or r.get('urdf_joint')
                               for r in rows}}
        break
    _CACHE[(str(robots_dir), robot_type)] = (time.time(), out)
    return out


def capabilities(robot_type, joint_names=None, custom=None, serial=False, topics=None):
    """The robot's capability profile. joint_names: the robot server's joint order;
    custom: custom_profile(); serial: RawBot Studio can read its arms over serial;
    topics: {'leader': ..., 'follower': ...} of the robot configuration."""
    topics = topics or {}
    if custom and custom.get('joints'):
        joints = [dict(j) for j in custom['joints']]
        source = custom['source']
    else:
        joints = [{'name': n, 'type': 'revolute', 'lower': None, 'upper': None}
                  for n in (joint_names or [])]
        source = 'robot configuration' if joints else ''
    for j in joints:
        j['gripper'] = bool(GRIPPER_RE.search(j['name'] or ''))
        ok = isinstance(j.get('lower'), (int, float)) and isinstance(j.get('upper'), (int, float))
        j['limits'] = [float(j['lower']), float(j['upper'])] if ok and j['lower'] < j['upper'] \
            else None
    names = [j['name'] for j in joints]
    gripper = next((j['name'] for j in joints if j['gripper']), None)
    n = len(joints)

    def ctl(ok, reason='', short=''):
        return {'available': bool(ok), 'reason': '' if ok else reason,
                'short': '' if ok else short}
    no_joints = ('The joint names of this robot are not known yet (robot configuration not '
                 'loaded).', 'joints unknown')
    controls = {
        'gui': ctl(n, *no_joints),
        'keyboard': ctl(0 < n <= KEYBOARD_MAX, *(no_joints if not n else (
            'Keyboard control covers up to %d joints (this robot has %d).' % (KEYBOARD_MAX, n),
            '> %d joints' % KEYBOARD_MAX))),
        'gamepad': ctl(0 < n <= GAMEPAD_MAX, *(no_joints if not n else (
            'A gamepad covers up to %d joints (this robot has %d).' % (GAMEPAD_MAX, n),
            '> %d joints' % GAMEPAD_MAX))),
        'script': ctl(n, *no_joints),
        'replay': ctl(n, *no_joints),
    }
    return {
        'robot_type': robot_type, 'source': source, 'joints': joints, 'names': names,
        'dof': n, 'gripper': gripper,
        'limits': {j['name']: j['limits'] for j in joints if j['limits']},
        'sim': ctl(n, *no_joints),
        'serial': bool(serial), 'leader_topic': topics.get('leader'),
        'follower_topic': topics.get('follower'), 'controls': controls,
    }


def mapping(source_names, target_names, gripper=None, pairs=None):
    """How a source (leader) maps onto a target: by joint name, or by the operator's pairs
    (target joint -> source joint) for robots whose joints are named differently."""
    src, tgt = list(source_names or []), list(target_names or [])
    pairs = pairs or {}
    via = {n: pairs.get(n, n) for n in tgt}
    matched = [n for n in tgt if via[n] in src]
    return {'matched': len(matched), 'total': len(tgt),
            'missing': [n for n in tgt if via[n] not in src],
            'extra': [n for n in src if n not in via.values()],
            'gripper': None if not gripper else (gripper in tgt and via[gripper] in src or
                                                 gripper in src)}
