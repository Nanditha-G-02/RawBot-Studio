# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""A simulated parallel gripper for a robot model that has none.

It is added to the robot's URDF (resolved.urdf) when the model is validated, so the 3D view,
the MuJoCo physics and the joint table all see the same robot. Nothing is sent to a real
robot: the leader arm has no gripper of its own, so the operator opens and closes this one
with a key or button in RawBot Studio, and its opening is recorded like any other joint.

One actuated joint, ``sim_gripper`` (prismatic, metres: 0 = open, ``travel`` = closed on
nothing), moves the left finger inwards; the right finger follows it with a <mimic>. Both
fingers have collision boxes, so a cube is held by contact (friction) in MuJoCo.
"""

import math
import re
import xml.etree.ElementTree as ET

JOINT = 'sim_gripper'
FOLLOWER_JOINT = 'sim_gripper_mimic'
FOLLOWER_JOINT_3 = 'sim_gripper_mimic2'     # the third finger of the three-finger gripper
ROS_NAME = 'gripper'
GRIPPER_LINKS = ('sim_gripper_base', 'sim_finger_left', 'sim_finger_right', 'sim_finger_c',
                 'sim_tcp')
DEFAULTS = {'fingers': 2, 'opening': 0.08, 'finger_length': 0.06, 'finger_depth': 0.02,
            'finger_thick': 0.008, 'body_length': 0.04, 'xyz': [0.0, 0.0, 0.0], 'rpy': [0.0, 0.0, 0.0]}
_FINGER_MASS = 0.03
_BASE_MASS = 0.15


class GripperError(ValueError):
    pass


def _f(v, lo, hi, what):
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise GripperError('%s must be a number.' % what)
    if not lo <= v <= hi:
        raise GripperError('%s must be between %g and %g.' % (what, lo, hi))
    return v


def options(raw):
    """Clean gripper options (metres, radians); raises GripperError."""
    raw = raw or {}
    try:
        fingers = int(raw.get('fingers', DEFAULTS['fingers']))
    except (TypeError, ValueError):
        fingers = 0
    if fingers not in (2, 3):
        raise GripperError('Fingers must be 2 (parallel) or 3 (tripod).')
    o = {
        'fingers': fingers,
        'opening': _f(raw.get('opening', DEFAULTS['opening']), 0.02, 0.25, 'Opening (m)'),
        'finger_length': _f(raw.get('finger_length', DEFAULTS['finger_length']), 0.02, 0.2,
                            'Finger length (m)'),
        'finger_depth': _f(raw.get('finger_depth', DEFAULTS['finger_depth']), 0.005, 0.08,
                           'Finger width (m)'),
        'finger_thick': _f(raw.get('finger_thick', DEFAULTS['finger_thick']), 0.003, 0.03,
                           'Finger thickness (m)'),
        'body_length': _f(raw.get('body_length', DEFAULTS['body_length']), 0.01, 0.15,
                          'Body length (m)'),
        'xyz': [_f(v, -0.5, 0.5, 'Offset') for v in (raw.get('xyz') or DEFAULTS['xyz'])][:3],
        'rpy': [_f(v, -2 * math.pi, 2 * math.pi, 'Rotation') for v in
                (raw.get('rpy') or DEFAULTS['rpy'])][:3],
    }
    if len(o['xyz']) != 3 or len(o['rpy']) != 3:
        raise GripperError('Offset and rotation need three numbers each.')
    link = raw.get('attach_link')
    if link:
        if not re.fullmatch(r'[A-Za-z0-9_.\-]{1,80}', str(link)):
            raise GripperError('Attach link is not a valid link name.')
        o['attach_link'] = str(link)
    return o


def travel(o):
    """How far each finger moves from open to closed (m): the value range of the joint."""
    return o['opening'] / 2.0


def tip_link(urdf_text):
    """The link the gripper belongs on: the child of the last movable joint of the arm (a
    chain without gripper parts), or None if the model has no movable joint."""
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return None
    joints = root.findall('joint')
    kids, parent_of, order = {}, {}, []
    for j in joints:
        p, c = j.find('parent'), j.find('child')
        if p is None or c is None:
            continue
        kids.setdefault(p.get('link'), []).append(j)
        parent_of[c.get('link')] = p.get('link')
    top = [l.get('name') for l in root.findall('link') if l.get('name') not in parent_of]
    best, depth_best = None, -1

    def walk(link, depth, last):
        nonlocal best, depth_best
        for j in kids.get(link, []):
            movable = j.get('type') in ('revolute', 'continuous', 'prismatic') and \
                j.find('mimic') is None
            child = j.find('child').get('link')
            nl = child if movable else last
            if movable and depth + 1 > depth_best:
                best, depth_best = child, depth + 1
            if depth < 64:
                walk(child, depth + 1 if movable else depth, nl)
    for t in top:
        walk(t, 0, None)
    return best


def has_gripper(urdf_text):
    return JOINT in urdf_text and 'sim_gripper_base' in urdf_text


def _box(x, y, z):
    return '<geometry><box size="%.5f %.5f %.5f"/></geometry>' % (x, y, z)


def _inertial(m, x, y, z):
    return ('<inertial><origin xyz="0 0 0"/><mass value="%g"/><inertia ixx="%.3e" ixy="0" '
            'ixz="0" iyy="%.3e" iyz="0" izz="%.3e"/></inertial>' % (
                m, m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12))


def _cyl(r, h):
    return '<geometry><cylinder radius="%.5f" length="%.5f"/></geometry>' % (r, h)


def _xml_three(o, attach_link):
    """Three fingers 120 degrees apart around the gripper axis, each sliding radially. One
    actuated joint and two mimics, so the three squeeze with the same stroke and centre an
    object of any outline (cube, cylinder, ball) on the axis."""
    t = travel(o)
    fl, fd, ft, bl = o['finger_length'], o['finger_depth'], o['finger_thick'], o['body_length']
    r0 = o['opening'] / 2 + ft / 2               # finger centre when open
    br = o['opening'] / 2 + ft + 0.012           # body radius
    names = [('a', JOINT, None), ('b', FOLLOWER_JOINT, JOINT), ('c', FOLLOWER_JOINT_3, JOINT)]
    out = [
        '<!-- simulated three-finger gripper (RawBot Studio): not part of the real robot -->',
        '<link name="sim_gripper_base"><visual><origin xyz="0 0 %.5f"/>%s'
        '<material name="sim_gripper_body"><color rgba="0.82 0.83 0.86 1"/></material></visual>'
        '<collision><origin xyz="0 0 %.5f"/>%s</collision>%s</link>' % (
            bl / 2, _cyl(br, bl), bl / 2, _cyl(br, bl), _inertial(_BASE_MASS, 2 * br, 2 * br, bl)),
        '<joint name="sim_gripper_mount" type="fixed"><parent link="%s"/>'
        '<child link="sim_gripper_base"/><origin xyz="%s" rpy="%s"/></joint>' % (
            attach_link, ' '.join('%.5f' % v for v in o['xyz']),
            ' '.join('%.5f' % v for v in o['rpy'])),
    ]
    for k, (tag, joint, leader) in enumerate(names):
        yaw = k * 2 * math.pi / 3
        out.append(
            '<link name="sim_finger_%s"><visual><origin xyz="0 0 %.5f"/>%s'
            '<material name="sim_gripper_finger"><color rgba="0.12 0.12 0.14 1"/></material>'
            '</visual><collision><origin xyz="0 0 %.5f"/>%s</collision>%s</link>' % (
                tag, fl / 2, _box(ft, fd, fl), fl / 2, _box(ft, fd, fl),
                _inertial(_FINGER_MASS, ft, fd, fl)))
        out.append(
            '<joint name="%s" type="prismatic"><parent link="sim_gripper_base"/>'
            '<child link="sim_finger_%s"/><origin xyz="%.5f %.5f %.5f" rpy="0 0 %.5f"/>'
            '<axis xyz="-1 0 0"/><limit lower="0" upper="%.5f" effort="40" velocity="0.2"/>%s'
            '</joint>' % (joint, tag, r0 * math.cos(yaw), r0 * math.sin(yaw), bl, yaw, t,
                          '<mimic joint="%s" multiplier="1" offset="0"/>' % leader
                          if leader else ''))
    out += [
        '<link name="sim_tcp"/>',
        '<joint name="sim_tcp_joint" type="fixed"><parent link="sim_gripper_base"/>'
        '<child link="sim_tcp"/><origin xyz="0 0 %.5f"/></joint>' % (bl + fl * 0.7),
    ]
    return '\n  '.join(out)


def xml(o, attach_link):
    """The URDF elements of the gripper, to be placed before </robot>."""
    if o.get('fingers') == 3:
        return _xml_three(o, attach_link)
    t = travel(o)
    fl, fd, ft, bl = o['finger_length'], o['finger_depth'], o['finger_thick'], o['body_length']
    bw = o['opening'] + 2 * ft + 0.02          # body width across the fingers
    # finger centre when open: its inner face at +-opening/2
    cy = o['opening'] / 2 + ft / 2

    def finger(side, sign):
        return (
            '<link name="sim_finger_%s"><visual><origin xyz="0 0 %.5f"/>%s'
            '<material name="sim_gripper_finger"><color rgba="0.12 0.12 0.14 1"/></material>'
            '</visual><collision><origin xyz="0 0 %.5f"/>%s</collision>%s</link>' % (
                side, fl / 2, _box(fd, ft, fl), fl / 2, _box(fd, ft, fl),
                _inertial(_FINGER_MASS, fd, ft, fl)))

    out = [
        '<!-- simulated gripper (RawBot Studio): not part of the real robot -->',
        '<link name="sim_gripper_base"><visual><origin xyz="0 0 %.5f"/>%s'
        '<material name="sim_gripper_body"><color rgba="0.82 0.83 0.86 1"/></material></visual>'
        '<collision><origin xyz="0 0 %.5f"/>%s</collision>%s</link>' % (
            bl / 2, _box(fd + 0.01, bw, bl), bl / 2, _box(fd + 0.01, bw, bl),
            _inertial(_BASE_MASS, fd + 0.01, bw, bl)),
        '<joint name="sim_gripper_mount" type="fixed"><parent link="%s"/>'
        '<child link="sim_gripper_base"/><origin xyz="%s" rpy="%s"/></joint>' % (
            attach_link, ' '.join('%.5f' % v for v in o['xyz']),
            ' '.join('%.5f' % v for v in o['rpy'])),
        finger('left', 1), finger('right', -1),
        '<joint name="%s" type="prismatic"><parent link="sim_gripper_base"/>'
        '<child link="sim_finger_left"/><origin xyz="0 %.5f %.5f" rpy="0 0 0"/>'
        '<axis xyz="0 -1 0"/><limit lower="0" upper="%.5f" effort="40" velocity="0.2"/></joint>' % (
            JOINT, cy, bl, t),
        '<joint name="%s" type="prismatic"><parent link="sim_gripper_base"/>'
        '<child link="sim_finger_right"/><origin xyz="0 %.5f %.5f" rpy="0 0 0"/>'
        '<axis xyz="0 1 0"/><limit lower="0" upper="%.5f" effort="40" velocity="0.2"/>'
        '<mimic joint="%s" multiplier="1" offset="0"/></joint>' % (
            FOLLOWER_JOINT, -cy, bl, t, JOINT),
        '<link name="sim_tcp"/>',
        '<joint name="sim_tcp_joint" type="fixed"><parent link="sim_gripper_base"/>'
        '<child link="sim_tcp"/><origin xyz="0 0 %.5f"/></joint>' % (bl + fl * 0.7),
    ]
    return '\n  '.join(out)


def attach(urdf_text, raw):
    """URDF text with the gripper added, and the options used. The text is returned unchanged
    (and options None) when it already has a gripper of this kind."""
    o = options(raw)
    if has_gripper(urdf_text):
        raise GripperError('This model already has the simulated gripper.')
    link = o.get('attach_link') or tip_link(urdf_text)
    if not link:
        raise GripperError('The model has no movable joint to put a gripper on.')
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        raise GripperError('The robot model is not valid XML.')
    names = {l.get('name') for l in root.findall('link')}
    own = [j.get('name') for j in root.findall('joint')
           if j.get('type') in ('revolute', 'continuous', 'prismatic') and
           re.search(r'grip|finger|jaw|claw|pinch', j.get('name') or '', re.I)]
    if own:
        raise GripperError('The model already has a gripper joint (%s).' % own[0])
    if link not in names:
        raise GripperError('The model has no link "%s".' % link)
    o['attach_link'] = link
    idx = urdf_text.rfind('</robot>')
    if idx < 0:
        raise GripperError('The robot model does not look like a URDF.')
    return urdf_text[:idx] + '  ' + xml(o, link) + '\n' + urdf_text[idx:], o


def config_row(o, motor_id):
    """The joint row for the robot configuration: the opening in metres, 0 = open."""
    return {'urdf_joint': JOINT, 'ros_joint': ROS_NAME, 'motor_id': motor_id, 'direction': 1,
            'offset': 0.0, 'units': 'm', 'enabled': True, 'soft_lower': 0.0,
            'soft_upper': round(travel(o), 5)}
