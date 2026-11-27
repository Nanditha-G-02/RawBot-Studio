# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""The simulated follower's world: the robot from its URDF (reach, joint mapping), the scene
objects, the simulation backends (kinematic, MuJoCo) and the MuJoCo worker.

The MuJoCo tests run when a Python with the ``mujoco`` package is available
(PHYSIOS_SIM_PYTHON, or var/sim-venv made by ./scripts/install.sh --with-mujoco)."""

import json
import math
import os
from pathlib import Path
import subprocess
import time

import pytest

from robot_fixtures import arm_urdf, stl_box
from physios_trainer.hardware import mujoco_worker, physics, robotcaps
from physios_trainer.hardware.simbot import OBJECTS_TOPIC, SIM_TOPIC, SimRobot, joint_mapping

REPO = Path(__file__).resolve().parents[2]


def custom_robot(root, rid='Demo-Arm', rows=None):
    """A registered custom robot as the robot store leaves it: profile, resolved URDF, meshes."""
    d = root / rid
    (d / 'files' / 'meshes').mkdir(parents=True)
    for n in ('base', 'l1', 'l2', 'l3'):
        (d / 'files' / 'meshes' / ('%s.stl' % n)).write_bytes(stl_box(0.04, 0.04, 0.08))
    (d / 'resolved.urdf').write_text(arm_urdf())
    joints = [{'name': 'j1', 'type': 'revolute', 'lower': -1.57, 'upper': 1.57},
              {'name': 'j2', 'type': 'revolute', 'lower': -1.57, 'upper': 1.57},
              {'name': 'j3', 'type': 'continuous', 'lower': None, 'upper': None}]
    mesh_map = {'package://demo_arm/meshes/%s.stl' % n: 'meshes/%s.stl' % n
                for n in ('base', 'l1', 'l2', 'l3')}
    prof = {'id': rid, 'name': rid, 'state': 'registered',
            'validation': {'summary': {'joint_list': joints, 'mesh_map': mesh_map}},
            'config': {'joints': rows or [{'urdf_joint': 'j1', 'ros_joint': 'pan'},
                                          {'urdf_joint': 'j2', 'ros_joint': 'lift',
                                           'direction': -1, 'offset': 0.1},
                                          {'urdf_joint': 'j3', 'ros_joint': 'roll',
                                           'units': 'deg'}]}}
    (d / 'profile.json').write_text(json.dumps(prof))
    return rid.lower().replace('-', '_')


# ---- the robot model ---------------------------------------------------------------------------
def test_reach_skips_the_shoulder_height(tmp_path):
    p = tmp_path / 'r.urdf'
    p.write_text(arm_urdf())            # joints at z = 0.1 (shoulder), 0.1, 0.1
    assert robotcaps.chain_reach(p) == pytest.approx(0.2)


def test_custom_robot_carries_what_physics_needs(tmp_path):
    rt = custom_robot(tmp_path)
    prof = robotcaps.custom_profile(tmp_path, rt)
    sim = prof['sim']
    assert sim['urdf'].endswith('resolved.urdf') and sim['joints'] == ['j1', 'j2', 'j3']
    assert sim['reach'] == pytest.approx(0.2)
    assert all(os.path.isfile(v) for v in sim['mesh_map'].values())


def test_joint_mapping_converts_ros_values_to_the_urdf_and_back():
    m = joint_mapping([{'urdf_joint': 'j2', 'ros_joint': 'lift', 'direction': -1, 'offset': 0.1},
                       {'urdf_joint': 'j3', 'ros_joint': 'roll', 'units': 'deg'},
                       {'urdf_joint': 'off', 'enabled': False}])
    s = SimRobot('ws://x', sampler=lambda t: (None, [], []), bridge_factory=None, thread=False)
    s.mapping = m
    assert s.to_urdf('lift', 0.5) == ('j2', pytest.approx(-0.4))
    assert s.from_urdf('lift', -0.4) == pytest.approx(0.5)
    assert s.to_urdf('roll', 90)[1] == pytest.approx(math.pi / 2)
    assert s.from_urdf('roll', math.pi / 2) == pytest.approx(90)
    assert 'off' not in m and s.to_urdf('pan', 0.3) == ('pan', 0.3)


# ---- the scene ---------------------------------------------------------------------------------
class Pub:
    def __init__(self):
        self.sent = []
        self.alive = True

    def advertise(self, t, typ):
        pass

    def publish(self, t, msg):
        self.sent.append((t, msg))


def test_scene_objects_are_published_with_their_poses():
    s = SimRobot('ws://x', sampler=lambda t: (None, [], []), bridge_factory=lambda u: Pub(),
                 thread=False)
    s.ensure('demo', True, {'follower': '/f'}, names=['a', 'b'])
    s._publish()
    assert OBJECTS_TOPIC not in [t for t, _ in s.br.sent]      # no scene yet: nothing to draw
    v = s.command({'command': 'scene', 'items': [
        {'id': 'cube', 'role': 'object', 'x': 0.2, 'y': 0.05, 'size': 0.03, 'color': '#f00'},
        {'id': 'tray', 'role': 'target', 'x': 0.25, 'y': -0.1, 'yaw': 90, 'size': 0.12}]})
    assert v['scene']['rev'] == 1 and v['objects_topic'] == OBJECTS_TOPIC
    s.br.sent.clear()
    s._publish()
    objs = [json.loads(m['data']) for t, m in s.br.sent if t == OBJECTS_TOPIC]
    assert objs and objs[0]['backend'] == 'kinematic'
    cube, tray = objs[0]['items']
    assert cube['p'] == [0.2, 0.05, 0.0] and cube['q'] == [1.0, 0.0, 0.0, 0.0]
    assert tray['q'][0] == pytest.approx(math.cos(math.pi / 4))   # yaw 90 deg about z
    assert SIM_TOPIC in [t for t, _ in s.br.sent]
    with pytest.raises(ValueError):
        s.command({'command': 'scene', 'items': [{'id': 'x'}]})    # no position


# ---- backends ----------------------------------------------------------------------------------
def test_mujoco_is_offered_only_when_it_can_run(tmp_path):
    physics._probe.clear()
    b = {x['id']: x for x in physics.backends(str(tmp_path / 'nope' / 'python'), {'urdf': 'r'})}
    assert b['kinematic']['available']
    assert not b['mujoco']['available'] and 'install.sh --with-mujoco' in b['mujoco']['detail']
    # a Python without the package
    import sys
    physics._probe.clear()
    b = {x['id']: x for x in physics.backends(sys.executable, {'urdf': 'r'})}
    try:
        import mujoco  # noqa: F401
        has = True
    except ImportError:
        has = False
    assert b['mujoco']['available'] == has
    if not has:
        assert 'not installed' in b['mujoco']['detail']


def test_prepare_urdf_for_mujoco(tmp_path):
    (tmp_path / 'a.stl').write_bytes(stl_box())
    (tmp_path / 'b.dae').write_text('<COLLADA/>')
    (tmp_path / 'b.stl').write_bytes(stl_box())
    urdf = ('<robot name="r"><link name="base"><visual><geometry><mesh filename="pkg/a.stl"/>'
            '</geometry></visual></link><link name="l1"><collision><geometry>'
            '<mesh filename="pkg/b.dae"/></geometry></collision></link>'
            '<joint name="j" type="revolute"><parent link="base"/><child link="l1"/>'
            '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
            '<joint name="m" type="revolute"><parent link="l1"/><child link="l2"/>'
            '<mimic joint="j" multiplier="-1" offset="0.2"/></joint><link name="l2"/>'
            '<gazebo/><transmission name="t"/></robot>')
    text, notes, mimics = mujoco_worker.prepare_urdf(urdf, {
        'pkg/a.stl': str(tmp_path / 'a.stl'), 'pkg/b.dae': str(tmp_path / 'b.dae')})
    assert '<visual' not in text and '<gazebo' not in text and '<transmission' not in text
    assert str(tmp_path / 'a.stl') in text            # base: its visual mesh for contacts
    assert str(tmp_path / 'b.stl') in text            # DAE swapped for the sibling STL
    assert mimics == [('m', 'j', -1.0, 0.2)] and '<mimic' not in text
    assert notes == []


def sim_python():
    for p in (os.environ.get('PHYSIOS_SIM_PYTHON'), str(REPO / 'var' / 'sim-venv' / 'bin' /
                                                        'python')):
        if p and os.path.isfile(p) and subprocess.run([p, '-c', 'import mujoco'],
                                                      capture_output=True).returncode == 0:
            return p
    return None


@pytest.mark.skipif(not sim_python(), reason='no Python with mujoco (install.sh --with-mujoco)')
def test_mujoco_follows_every_joint_and_holds_the_objects(tmp_path):
    rt = custom_robot(tmp_path)
    sim = robotcaps.custom_profile(tmp_path, rt)['sim']
    physics._probe.clear()
    assert {b['id']: b for b in physics.backends(sim_python(), sim)}['mujoco']['available']
    b = physics.MujocoBackend(sim_python())
    items = [{'id': 'cube', 'role': 'object', 'x': 0.25, 'y': 0.1, 'yaw': 0, 'size': 0.03},
             {'id': 'tray', 'role': 'target', 'x': 0.25, 'y': -0.15, 'yaw': 0, 'size': 0.12}]
    try:
        b.sync(sim, items, {'j1': 0, 'j2': 0, 'j3': 0})
        t0 = time.time()
        while time.time() - t0 < 30 and not (b.loaded or b.error):
            time.sleep(0.1)
        assert b.loaded and not b.error, b.error
        assert b.loaded['joints'] == ['j1', 'j2', 'j3']
        for j, target in (('j1', 0.6), ('j2', -0.5), ('j3', 2.0)):
            q = {'j1': 0.0, 'j2': 0.0, 'j3': 0.0}
            q[j] = target
            b.set_targets(q)
            time.sleep(1.0)
            st = b.latest()
            assert st['q'][j] == pytest.approx(target, abs=0.02), (j, st['q'])
        st = b.latest()
        cube = next(o for o in st['objects'] if o['id'] == 'cube')
        assert cube['p'][0] == pytest.approx(0.25, abs=0.005) and abs(cube['p'][2]) < 0.005
        assert b.view()['live'] and b.view()['version']
    finally:
        b.stop()
