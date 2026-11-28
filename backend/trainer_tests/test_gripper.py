# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""The simulated gripper: added to a robot model that has none, driven by the operator (the
leader arm has no gripper), recorded as a joint, and holding a cube by contact in MuJoCo."""

import json
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from physios_trainer.hardware import physics, robotcaps  # noqa: E402
from physios_trainer.hardware.simbot import SimRobot, joint_mapping  # noqa: E402
from physios_trainer.robots import gripper, readiness as rd  # noqa: E402
from physios_trainer.robots.store import RobotError, RobotStore  # noqa: E402
import robot_fixtures as rf  # noqa: E402
from test_simulation import sim_python  # noqa: E402


@pytest.fixture()
def reg(tmp_path):
    st = RobotStore(tmp_path)
    rid = st.create({'name': 'Arm'})['id']
    st.add_zip(rid, rf.arm_zip())
    st.validate(rid)
    st.register(rid)
    return st, rid


def test_the_gripper_goes_on_the_last_arm_link():
    assert gripper.tip_link(rf.arm_urdf()) == 'l3'
    text, o = gripper.attach(rf.arm_urdf(), {'opening': 0.1})
    assert o['attach_link'] == 'l3' and o['opening'] == 0.1
    assert 'name="sim_gripper"' in text and '<mimic joint="sim_gripper"' in text
    with pytest.raises(gripper.GripperError, match='already'):
        gripper.attach(text, {})
    with pytest.raises(gripper.GripperError, match='between'):
        gripper.options({'opening': 5})
    with pytest.raises(gripper.GripperError, match='no link'):
        gripper.attach(rf.arm_urdf(), {'attach_link': 'nope'})


def test_a_model_with_its_own_gripper_is_left_alone():
    own = rf.arm_urdf().replace('name="j3"', 'name="gripper_joint"')
    with pytest.raises(gripper.GripperError, match='already has a gripper joint'):
        gripper.attach(own, {})


def test_adding_and_removing_it_updates_the_model_and_the_configuration(reg):
    st, rid = reg
    cfg = st.default_config(rid)
    st.save_config(rid, cfg)
    p = st.set_sim_gripper(rid, {'enabled': True, 'opening': 0.07})
    assert 'sim_gripper' in (st._dir(rid) / 'resolved.urdf').read_text()
    rows = {r['urdf_joint']: r for r in p['config']['joints']}
    # one row for the actuated joint; the mimic finger has no motor of its own
    assert 'sim_gripper' in rows and 'sim_gripper_mimic' not in rows
    g = rows['sim_gripper']
    assert g['ros_joint'] == 'gripper' and g['units'] == 'm'
    assert (g['soft_lower'], g['soft_upper']) == (0.0, 0.035)
    assert not [i for i in p['config_issues'] if i['level'] == 'error'], p['config_issues']
    assert [j['name'] for j in rd.movable_joints(p)].count('sim_gripper_mimic') == 0
    with pytest.raises(RobotError, match='already'):
        st.set_sim_gripper(rid, {'enabled': True})
    # a robot validated again (new upload of the same model) keeps the gripper
    assert 'sim_gripper' in st.validate(rid)['validation']['summary']['joint_list'][-2]['name']
    p = st.set_sim_gripper(rid, {'enabled': False})
    assert 'sim_gripper' not in (st._dir(rid) / 'resolved.urdf').read_text()
    assert [r['urdf_joint'] for r in p['config']['joints']] == ['j1', 'j2', 'j3']
    assert not [i for i in p['config_issues'] if i['level'] == 'error']


def test_it_needs_a_registered_model(tmp_path):
    st = RobotStore(tmp_path)
    rid = st.create({'name': 'Arm'})['id']
    with pytest.raises(RobotError, match='Register'):
        st.set_sim_gripper(rid, {'enabled': True})


def make_sim(opening=0.06):
    rows = [{'urdf_joint': 'j1', 'ros_joint': 'j1'},
            {'urdf_joint': 'sim_gripper', 'ros_joint': 'gripper', 'units': 'm'}]
    sim = SimRobot('ws://x', lambda t: (None, [], []), bridge_factory=None, thread=False)
    sim.ensure('r', True, {}, names=['j1', 'gripper'], limits={'gripper': (0.0, opening / 2)},
               mapping=joint_mapping(rows))
    return sim


def test_the_sim_gripper_opens_and_closes_without_taking_over_the_arm():
    sim = make_sim()
    sim.command({'command': 'drive', 'drive': 'leader'})
    assert sim.drive == 'leader' and sim.gripper_index() == 1
    assert sim.view()['gripper'] == {'joint': 'gripper', 'closed': False, 'value': 0.0,
                                     'own': True}
    sim.command({'command': 'gripper', 'state': 'close'})
    assert sim.target[1] == pytest.approx(0.03) and sim.drive == 'leader'
    for _ in range(30):
        sim.step(0.05)
    assert sim.view()['gripper']['closed'] and sim.view()['gripper']['value'] == 1.0
    sim.command({'command': 'gripper', 'state': 'toggle'})
    assert sim.target[1] == 0.0
    sim.command({'command': 'gripper', 'value': 0.5})
    assert sim.target[1] == pytest.approx(0.015)
    with pytest.raises(ValueError, match='state open'):
        sim.command({'command': 'gripper'})


def test_a_robot_without_a_gripper_says_so():
    sim = SimRobot('ws://x', lambda t: (None, [], []), bridge_factory=None, thread=False)
    sim.ensure('r', True, {}, names=['a', 'b'])
    assert 'gripper' not in sim.view()
    with pytest.raises(ValueError, match='no gripper'):
        sim.command({'command': 'gripper', 'state': 'close'})


LIFT_URDF = """<?xml version="1.0"?><robot name="lift">
<link name="base"/>
<link name="hand"><inertial><mass value="0.3"/><inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/></inertial>
 <collision><geometry><box size="0.02 0.02 0.02"/></geometry></collision></link>
<joint name="lift" type="prismatic"><parent link="base"/><child link="hand"/>
 <origin xyz="0.2 0 0.105"/><axis xyz="0 0 1"/><limit lower="0" upper="0.2" effort="100" velocity="1"/></joint>
</robot>"""


def test_the_three_finger_gripper_has_three_fingers_and_two_mimics():
    text, o = gripper.attach(rf.arm_urdf(), {'fingers': 3, 'opening': 0.1})
    assert o['fingers'] == 3
    for n in ('sim_finger_a', 'sim_finger_b', 'sim_finger_c', 'sim_gripper_mimic',
              'sim_gripper_mimic2'):
        assert 'name="%s"' % n in text
    assert text.count('<mimic joint="sim_gripper"') == 2
    assert gripper.has_gripper(text) and gripper.travel(o) == 0.05
    with pytest.raises(gripper.GripperError, match='Fingers'):
        gripper.options({'fingers': 4})
    # the two-finger default is unchanged
    assert gripper.options({})['fingers'] == 2


@pytest.mark.skipif(not sim_python(), reason='no Python with mujoco (install.sh --with-mujoco)')
@pytest.mark.parametrize('fingers,shape', [(2, 'cube'), (3, 'cube'), (3, 'cylinder'),
                                           (3, 'sphere')])
def test_mujoco_holds_a_cube_between_the_fingers_and_lifts_it(tmp_path, fingers, shape):
    d = tmp_path / 'Lift'
    d.mkdir()
    # fingers point down (rotated half a turn about x); tips end 5 mm above the table
    text, o = gripper.attach(LIFT_URDF, {'opening': 0.08, 'fingers': fingers,
                                          'rpy': [3.14159265, 0, 0]})
    (d / 'resolved.urdf').write_text(text)
    joints = [{'name': 'lift', 'type': 'prismatic', 'lower': 0.0, 'upper': 0.2},
              {'name': 'sim_gripper', 'type': 'prismatic', 'lower': 0.0, 'upper': 0.04}]
    rows = [{'urdf_joint': 'lift', 'ros_joint': 'lift', 'units': 'm'},
            {'urdf_joint': 'sim_gripper', 'ros_joint': 'gripper', 'units': 'm'}]
    (d / 'profile.json').write_text(json.dumps({
        'id': 'Lift', 'state': 'registered', 'config': {'joints': rows},
        'validation': {'summary': {'joint_list': joints, 'mesh_map': {}}}}))
    sim = robotcaps.custom_profile(tmp_path, 'lift')['sim']
    assert sim['joints'] == ['lift', 'sim_gripper']
    physics._probe.clear()
    b = physics.MujocoBackend(sim_python())
    cube = {'id': 'cube', 'role': 'object', 'x': 0.2, 'y': 0.0, 'yaw': 0, 'size': 0.03,
            'shape': shape}

    def settle(q, s):
        b.set_targets(q)
        time.sleep(s)
        return b.latest()
    try:
        b.sync(sim, [cube], {'lift': 0.0, 'sim_gripper': 0.0})
        t0 = time.time()
        while time.time() - t0 < 30 and not (b.loaded or b.error):
            time.sleep(0.1)
        assert b.loaded and not b.error, b.error
        # lowered over the cube with the gripper open, close, lift
        st = settle({'lift': 0.0, 'sim_gripper': 0.0}, 1.0)
        z0 = next(x for x in st['objects'] if x['id'] == 'cube')['p'][2]
        assert z0 < 0.005
        st = settle({'lift': 0.0, 'sim_gripper': 0.04}, 1.5)
        assert 0.01 < st['q']['sim_gripper'] < 0.039, st['q']     # stopped by the cube
        st = settle({'lift': 0.15, 'sim_gripper': 0.04}, 2.0)
        z = next(x for x in st['objects'] if x['id'] == 'cube')['p'][2]
        assert z > 0.1, 'the cube was not lifted (z=%.3f)' % z
        # opened: the cube is let go and falls back to the table
        st = settle({'lift': 0.15, 'sim_gripper': 0.0}, 1.5)
        z = next(x for x in st['objects'] if x['id'] == 'cube')['p'][2]
        assert z < 0.02, 'the cube was not released (z=%.3f)' % z
    finally:
        b.stop()
