# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Hardware-aware Planning: action source, target robot and observation per configuration,
the simulated robot, and how a demonstration is recorded in each of the five setups."""

import math
import time

import pytest

from hw_fixtures import FakeRos, HwWorld
from physios_trainer.environment import Environments
from physios_trainer.hardware.manager import Hardware, HardwareError
from physios_trainer.hardware.simbot import SIM_TOPIC, SimRobot

RT = 's101_dataset'
TOPICS = {'follower': '/joint_states', 'leader': '/leader/joint_trajectory'}
CAMS = {'global': '/camera/color/image_raw/compressed'}
J = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


class Pub:
    def __init__(self):
        self.sent = []
        self.alive = True

    def advertise(self, t, typ):
        pass

    def publish(self, t, msg):
        self.sent.append((t, msg))


@pytest.fixture
def world(tmp_path, monkeypatch):
    w = HwWorld(tmp_path / 'usb')
    w.apply_env(monkeypatch)
    yield w
    w.close()


@pytest.fixture
def hw(world, tmp_path):
    h = Hardware(tmp_path / 'hw', Environments(), monitor=False)
    h.ros = FakeRos()
    h.ros.publish(CAMS['global'], [], [], publisher='/camera_node')
    h.setup(RT, topics=TOPICS, cameras=CAMS, joints=J)
    yield h
    h.stop()


def register(world, hw, roles=('leader', 'follower')):
    serial = {'leader': ('ttyACM0', 'LEAD-A'), 'follower': ('ttyACM1', 'FOLL-B')}
    for r in roles:
        world.plug(serial[r][0], serial=serial[r][1])
    st = hw.detect(RT)
    keys = {u['port']['device']: u['key'] for u in st['unassigned']}
    for r in roles:
        hw.assign(RT, keys[world.device(serial[r][0])], r)
    for r in roles:
        world.hold(serial[r][0])
    hw.ports_at = 0


def opts(st, kind):
    return {o['id']: o for o in st['planning'][kind]}


def live(hw, leader=True, follower=True):
    if follower:
        hw.ros.publish('/joint_states', J, [0.1] * 6)
    if leader:
        hw.ros.publish('/leader/joint_trajectory', J, [0.2] * 6)


# --------------------------------------------------------------------- configurations ---
def test_leader_follower_defaults_to_teleoperation(world, hw):
    register(world, hw)
    live(hw)
    st = hw.status(RT)
    p = st['planning']
    assert p['config'] == 'leader_follower'
    assert p['selection'] == {'action': 'leader', 'target': 'follower',
                              'observation': 'follower'}
    assert p['record']['available'] and p['record']['action_param'] == 'leader'
    a = opts(st, 'action_sources')
    assert a['leader']['available']
    assert not a['script']['available'] and a['script']['short'] == 'simulation off'
    assert not a['hand_guided']['available']
    o = opts(st, 'observations')
    assert not o['sim']['available']
    assert p['cameras'] == [{'name': 'global', 'topic': CAMS['global'], 'live': True}]
    assert st['recording_gate'] == p['record']


def test_leader_only_records_through_the_simulated_robot(world, hw):
    register(world, hw, ('leader',))
    hw.setup(RT, mode='leader_only')
    live(hw, follower=False)
    st = hw.status(RT)
    p = st['planning']
    assert p['config'] == 'leader_only' and not st['sim']['running']
    assert not p['record']['available'] and 'Simulated twin' in p['record']['reason']
    st = hw.setup(RT, sim_twin=True)
    p = st['planning']
    assert p['config'] == 'physical_sim'
    assert p['selection'] == {'action': 'leader', 'target': 'sim', 'observation': 'sim'}
    assert st['sim']['state_topic'] == '/joint_states'      # free: no physical follower
    assert p['record']['available'] and p['record']['action_param'] == 'leader'
    # the leader drives the simulation
    hw.sim.command({'command': 'drive', 'drive': 'leader'})
    for _ in range(40):
        hw.sim.step(1 / 30)
    assert all(abs(v - 0.2) < 1e-6 for v in hw.sim.pos)
    # the simulated robot's own messages on /joint_states are not a follower arm: no upgrade
    hw.ros.publish('/joint_states', J, hw.sim.pos, publisher=None)
    st = hw.status(RT)
    assert st['arms']['follower']['state'] == 'unused' and st['mode'] == 'leader_only'
    assert hw._ros_live(RT, 'follower', hw.reg.setup(RT))[0] is False
    # a real follower appears (bridge.py publishes the topic): the sim yields the topic
    hw.ros.pubs['/joint_states'] = ['/so101_bridge']
    st = hw.status(RT)
    assert st['sim']['state_topic'] is None
    assert '/so101_bridge' in st['sim']['own_reason']


def test_follower_only_hand_guided_and_direct_control(world, hw):
    register(world, hw, ('follower',))
    hw.setup(RT, mode='follower_only')
    live(hw, leader=False)
    hw.report_action_source(RT, True)
    st = hw.status(RT)
    p = st['planning']
    assert p['selection']['action'] == 'hand_guided'
    assert p['record']['available'] and p['record']['action_param'] == 'follower'
    a = opts(st, 'action_sources')
    assert a['gui']['available'] and a['keyboard']['available'] and a['gamepad']['available']
    assert not a['leader']['available']
    # GUI control of the physical follower: its supervised commands are the action
    st = hw.setup(RT, selection={'action': 'gui', 'target': 'follower'})
    p = st['planning']
    assert p['selection']['target'] == 'follower'
    assert not p['record']['available'] and 'direct control' in p['record']['reason']
    assert p['record']['action_param'] == 'leader'
    # scripts and replays never drive the physical arm
    st = hw.setup(RT, selection={'action': 'script'})
    t = opts(st, 'targets')
    assert not a['script']['available'] or not t['follower']['available']


def test_simulation_only_works_without_any_arm(world, hw):
    st = hw.setup(RT, mode='simulation')
    p = st['planning']
    assert p['config'] == 'simulation' and st['sim']['running']
    assert p['selection'] == {'action': 'gui', 'target': 'sim', 'observation': 'sim'}
    assert p['record']['available'] and p['record']['action_param'] == 'follower'
    a = opts(st, 'action_sources')
    for k in ('gui', 'keyboard', 'gamepad', 'script', 'replay'):
        assert a[k]['available'], k
    assert not a['leader']['available'] and not a['hand_guided']['available']
    assert not opts(st, 'targets')['follower']['available']
    # cameras of the robot configuration must publish to record
    hw.ros.pubs[CAMS['global']] = []
    st = hw.status(RT)
    assert not st['planning']['record']['available']
    assert 'Camera global' in st['planning']['record']['reason']
    hw.ros.unknown.add(CAMS['global'])      # rosapi cannot tell: not blocking
    assert hw.status(RT)['planning']['record']['available']


def test_physical_plus_simulated_mirror_and_comparison(world, hw):
    register(world, hw)
    live(hw)
    st = hw.setup(RT, sim_twin=True)
    p = st['planning']
    assert p['config'] == 'physical_sim'
    assert st['sim']['state_topic'] is None          # the physical follower owns the state
    assert not opts(st, 'observations')['sim']['available']
    hw.sim.command({'command': 'drive', 'drive': 'follower'})
    for _ in range(30):
        hw.sim.step(1 / 30)
    st = hw.status(RT)
    c = st['planning']['compare']
    assert c and c['max_abs'] < 1e-6 and len(c['joints']) == 6
    # the leader drives the simulation; the physical follower stays the recorded state
    st = hw.setup(RT, selection={'action': 'leader', 'target': 'sim'})
    p = st['planning']
    assert p['selection']['observation'] == 'follower'
    assert p['record']['available'] and p['record']['action_param'] == 'leader'


def test_choice_that_no_longer_fits_falls_back_to_the_default(world, hw):
    st = hw.setup(RT, mode='simulation', selection={'action': 'script', 'target': 'sim'})
    assert st['planning']['selection']['action'] == 'script'
    register(world, hw)
    live(hw)
    st = hw.setup(RT, mode='leader_follower', sim_twin=False)
    assert st['planning']['selection']['action'] == 'leader'
    # the configuration changed: the old choice is gone, not just hidden while unavailable
    assert hw.reg.setup(RT)['selection'] == {}
    hw.setup(RT, selection={'action': 'gui'})
    st = hw.setup(RT, mode='leader_follower')              # same configuration: kept
    assert hw.reg.setup(RT)['selection']['action'] == 'gui'


def test_sim_commands_need_the_simulated_robot(world, hw):
    with pytest.raises(HardwareError):
        hw.sim_command(RT, {'command': 'home'})
    hw.setup(RT, mode='simulation')
    st = hw.sim_command(RT, {'command': 'targets', 'targets': {'elbow_flex': 9}})
    assert st['sim']['target'][2] == pytest.approx(math.pi)       # clamped to the limit
    with pytest.raises(HardwareError):
        hw.sim_command(RT, {'command': 'targets', 'targets': {'nope': 1}})
    st = hw.estop('test')
    assert 'simulated robot' not in st['stopped']                 # manual: nothing to stop
    hw.sim_command(RT, {'command': 'script', 'points': [[0.1] * 6]})
    assert 'simulated robot' in hw.estop('test')['stopped']
    assert hw.sim.drive == 'manual'


# ------------------------------------------------------------------ simulated robot ---
def sim(**kw):
    s = SimRobot('ws://x', sampler=lambda t: (None, [], []), bridge_factory=lambda u: Pub(),
                 thread=False)
    s.ensure(RT, True, TOPICS, names=J, **kw)
    return s


def test_sim_tracks_targets_at_the_joint_speed():
    s = sim()
    s.command({'command': 'targets', 'targets': {'shoulder_pan': 1.0}})
    s.step(0.1)
    assert s.pos[0] == pytest.approx(0.15) and s.vel[0] == pytest.approx(1.5)
    for _ in range(20):
        s.step(0.1)
    assert s.pos[0] == pytest.approx(1.0) and s.vel[0] == 0


def test_sim_held_velocity_expires():
    s = sim()
    s.command({'command': 'velocity', 'velocity': {'gripper': 1}})
    s.step(0.1)
    s.step(0.1)
    assert s.target[5] > 0
    s.hold[5] = (1, time.time() - 1)   # the key was released / the page closed
    t = s.target[5]
    s.step(0.1)
    assert s.target[5] == t


def test_sim_script_and_replay():
    s = sim()
    s.command({'command': 'script', 'points': [[0.05] * 6, [0.0] * 6], 'hold_s': 0})
    for _ in range(30):
        s.step(1 / 30)
    assert s.view()['script']['done'] and max(map(abs, s.pos)) < 1e-9
    s.command({'command': 'replay', 't': [0, 1, 2], 'names': J[:2],
               'rows': [[0, 0], [0.5, -0.5], [1, -1]], 'speed': 4})
    for _ in range(60):
        s.step(1 / 30)
    v = s.view()
    assert v['replay']['done']
    assert s.pos[0] == pytest.approx(1.0) and s.pos[1] == pytest.approx(-1.0)
    assert s.pos[2] == 0      # joints the trajectory doesn't have stay put


def test_sim_publishes_state_topic_only_when_allowed():
    s = sim()
    s._publish()
    assert [t for t, _ in s.br.sent] == [SIM_TOPIC]
    msg = s.br.sent[0][1]
    assert msg['name'] == J and len(msg['velocity']) == 6
    s.ensure(RT, True, TOPICS, names=J, state_topic='/joint_states')
    s.br.sent.clear()
    s._publish()
    assert [t for t, _ in s.br.sent] == [SIM_TOPIC, '/joint_states']
    assert TOPICS['leader'] not in [t for t, _ in s.br.sent]
    s.ensure(RT, False, TOPICS)
    s.br.sent.clear()
    s._publish()
    assert not s.br.sent


def test_selection_sets_what_drives_the_simulated_robot(world, hw):
    """Leader → simulated robot: the leader drives it; physical follower as target: the
    simulated robot mirrors the follower; GUI / script / replay: the operator drives it."""
    register(world, hw)
    live(hw)
    st = hw.setup(RT, sim_twin=True)
    assert st['sim']['drive'] == 'follower'                       # physical-to-simulation
    st = hw.setup(RT, selection={'action': 'leader', 'target': 'sim'})
    assert st['sim']['drive'] == 'leader'
    st = hw.setup(RT, selection={'action': 'script', 'target': 'sim'})
    assert st['sim']['drive'] == 'manual'
    hw.sim_command(RT, {'command': 'script', 'points': [[0.3] * 6]})
    assert hw.status(RT)['sim']['drive'] == 'script'              # a running script is kept
    st = hw.setup(RT, selection={'action': 'gui'})
    assert st['sim']['drive'] == 'manual'                         # ... until GUI is chosen


def test_leader_capture_replays_in_simulation(world, hw):
    hw.setup(RT, mode='simulation')
    rec = hw.save_capture(RT, {'role': 'leader', 'names': J,
                               'samples': [[0.0, [0.0] * 6], [1.0, [0.4] * 6]]})
    st = hw.sim_command(RT, {'command': 'replay', 'capture': rec['id'], 'speed': 4})
    assert st['sim']['drive'] == 'replay' and rec['id'] in st['sim']['replay']['label']
    for _ in range(30):
        hw.sim.step(1 / 30)
    assert all(abs(v - 0.4) < 1e-6 for v in hw.sim.pos)
    with pytest.raises(HardwareError):
        hw.sim_command(RT, {'command': 'replay', 'capture': '../x'})


def test_simulated_trajectory_capture_saves_and_replays(world, hw):
    hw.setup(RT, mode='simulation')
    hw.sim_command(RT, {'command': 'capture_start'})
    hw.sim_command(RT, {'command': 'targets', 'targets': {'shoulder_pan': 0.5}})
    for _ in range(30):
        hw.sim.step(1 / 30)
    assert hw.status(RT)['sim']['capture']['samples'] == 30
    st = hw.sim_command(RT, {'command': 'capture_stop'})
    cap = st['saved_capture']
    assert cap['role'] == 'sim' and cap['count'] == 30 and cap['names'] == J
    assert 'capture' not in st['sim']
    with pytest.raises(HardwareError):
        hw.sim_command(RT, {'command': 'capture_stop'})
    hw.sim_command(RT, {'command': 'home'})
    for _ in range(60):
        hw.sim.step(1 / 30)
    hw.sim_command(RT, {'command': 'replay', 'capture': cap['id'], 'speed': 4})
    for _ in range(60):
        hw.sim.step(1 / 30)
    assert hw.sim.pos[0] == pytest.approx(0.5, abs=1e-3)
