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
    # only a leader: the simulated follower is the execution target without any switch
    assert p['config'] == 'leader_sim' and p['auto'] and st['sim']['running']
    assert p['selection'] == {'action': 'leader', 'target': 'sim', 'observation': 'sim'}
    assert {c['id']: c['state'] for c in p['checks']}['mapping'] == 'ok'
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


# ------------------------------------------- Physical Leader + Simulated Follower ---
def serial_leader(world, hw):
    world.plug('ttyACM0', serial='LEAD-A')
    st = hw.detect(RT)
    hw.assign(RT, st['unassigned'][0]['key'], 'leader')
    st = hw.setup(RT, mode='leader_sim')
    st = hw.connect(RT, 'leader')
    assert st['arms']['leader']['source'] == 'serial'
    for _ in range(40):                      # first reading of the read-only session
        if hw._monitor_status(RT, 'leader').get('phase') == 'running':
            break
        time.sleep(0.1)
    return hw.status(RT)


def test_ticks_to_radians_and_joint_map():
    from physios_trainer.hardware.manager import clean_joint_map, ticks_to_rad
    assert ticks_to_rad(2048) == 0.0
    assert ticks_to_rad(3072) == pytest.approx(math.pi / 2)
    assert ticks_to_rad(1024, {'sign': -1, 'offset': 0.1}) == pytest.approx(math.pi / 2 + 0.1)
    assert clean_joint_map({'gripper': {'sign': -1, 'offset': '0.2'}, 'x': 3}) == {
        'gripper': {'sign': -1, 'offset': 0.2}}
    with pytest.raises(HardwareError):
        clean_joint_map({'gripper': {'offset': 9}})


def test_leader_sim_serial_leader_drives_the_simulated_follower(world, hw):
    """One physical arm read over serial (no bridge.py): the simulated follower reproduces it,
    its joints are the recorded state and the leader's are the recorded action."""
    st = serial_leader(world, hw)
    p = st['planning']
    assert p['config'] == 'leader_sim' and st['sim']['running']
    assert p['selection'] == {'action': 'leader', 'target': 'sim', 'observation': 'sim'}
    assert {r['id']: r['label'] for r in p['robots']}['sim'] == 'Simulated follower'
    t = opts(st, 'targets')
    assert not t['follower']['available'] and t['follower']['short'] == 'simulated follower'
    assert st['sim']['drive'] == 'leader' and st['sim']['state_topic'] == '/joint_states'
    assert p['record']['action_param'] == 'leader_state'
    # the leader moves: shoulder_pan (motor 1) to 3072 ticks = +pi/2
    world.set('ttyACM0', actual={'1': 3072, '2': 2048, '3': 2048, '4': 2048, '5': 2048,
                                 '6': 1024})
    time.sleep(0.4)
    for _ in range(40):
        hw.sim.step(1 / 30)
    li = hw.sim.view()['leader_in']
    assert li['source'] == 'serial' and not li['stale'] and li['ticks'][0] == 3072
    assert hw.sim.pos[0] == pytest.approx(math.pi / 2, abs=1e-3)
    assert hw.sim.pos[5] == pytest.approx(-math.pi / 2, abs=1e-3)      # gripper follows too
    st = hw.status(RT)
    assert st['planning']['record']['available'], st['planning']['record']['reason']
    assert st['capabilities']['record']['available']                    # the Hardware page agrees
    # the leader joints are published for the recorder, never on the command topic
    pub = Pub()
    hw.sim.br, hw.sim._advertised = pub, set()
    hw.sim.bridge_factory = lambda url: pub
    hw.sim._publish()
    topics = [t for t, _ in pub.sent]
    assert '/rawbot/leader/joint_states' in topics and '/joint_states' in topics
    assert '/leader/joint_trajectory' not in topics
    lead = dict(pub.sent)['/rawbot/leader/joint_states']
    assert lead['position'][0] == pytest.approx(math.pi / 2)
    # joint limits: clamped and reported
    hw.sim_command(RT, {'command': 'limits', 'limits': {'shoulder_pan': [-1.0, 1.0]}})
    hw.sim.step(1 / 30)
    assert hw.sim.target[0] == pytest.approx(1.0)
    assert 'shoulder_pan' in hw.sim.view()['leader_in']['at_limit']
    # joint map: direction flipped
    hw.setup(RT, joint_map={'shoulder_pan': {'sign': -1}})
    hw.sim_command(RT, {'command': 'limits', 'limits': {'shoulder_pan': [-3.0, 3.0]}})
    for _ in range(40):
        hw.sim.step(1 / 30)
    assert hw.sim.pos[0] == pytest.approx(-math.pi / 2, abs=1e-3)


def test_leader_sim_needs_a_robot_runtime_that_records_the_leader_state(world, hw):
    serial_leader(world, hw)
    hw.report_action_source(RT, False, 'leader_state')
    st = hw.status(RT)
    assert not st['planning']['record']['available']
    assert 'physios-docker.sh build robot' in st['planning']['record']['reason']
    hw.report_action_source(RT, True, 'leader_state')
    assert hw.status(RT)['planning']['record']['available']


def test_leader_sim_stale_leader_holds_and_blocks_recording(world, hw):
    serial_leader(world, hw)
    hw.disconnect(RT, 'leader')
    st = hw.status(RT)
    assert not st['planning']['record']['available']
    assert not opts(st, 'action_sources')['leader']['available']


def test_leader_sim_with_bridge_uses_the_leader_topic(world, hw):
    """With bridge.py publishing the leader, nothing changes in the recorder (action = the
    leader topic), and a physical follower appearing does not switch the configuration."""
    register(world, hw, ('leader',))
    hw.setup(RT, mode='leader_sim')
    live(hw, follower=False)
    st = hw.status(RT)
    assert st['planning']['record']['action_param'] == 'leader'
    assert st['planning']['record']['available']
    hw._auto_upgrade(RT, 'leader_sim', {'follower'})
    assert hw.status(RT)['mode'] == 'leader_sim'


def test_jog_stop_is_not_overwritten_by_a_heartbeat(tmp_path, hw):
    """Heartbeats and Stop come on parallel requests; once Stop is written it stays."""
    import json as _json
    import threading as _th

    class Proc:
        def poll(self):
            return None
    d = tmp_path / 'jog'
    d.mkdir()
    (d / 'control.json').write_text(_json.dumps({'command': 'run', 'heartbeat': 0}))
    hw.jog = {'id': 'jog-x', 'proc': Proc(), 'dir': d, 'robot_type': RT, 'started': 0}
    stop = _th.Event()

    def beat():
        while not stop.is_set():
            hw.jog_command('jog-x', {'command': 'heartbeat'})
    ts = [_th.Thread(target=beat) for _ in range(4)]
    for t in ts:
        t.start()
    time.sleep(0.05)
    hw.jog_command('jog-x', {'command': 'stop'})
    time.sleep(0.1)
    stop.set()
    for t in ts:
        t.join()
    assert _json.loads((d / 'control.json').read_text())['command'] == 'stop'
    hw.jog = None


# ------------------------------------------------------- robot-aware (any robot) ---
def write_custom(root, rid, joints, config=None):
    import json as _json
    d = root / rid
    d.mkdir(parents=True)
    prof = {'id': rid, 'name': rid, 'validation': {'summary': {'joint_list': joints}}}
    if config:
        prof['config'] = config
    (d / 'profile.json').write_text(_json.dumps(prof))


def test_capabilities_come_from_the_robot_not_its_model():
    from physios_trainer.hardware import robotcaps
    c = robotcaps.capabilities('any', ['j%d' % i for i in range(7)] + ['finger_joint'])
    assert c['dof'] == 8 and c['gripper'] == 'finger_joint'
    assert c['controls']['keyboard']['available']                 # up to 9 joints
    assert not c['controls']['gamepad']['available'] and c['controls']['gamepad']['short'] == '> 6 joints'
    c = robotcaps.capabilities('any', [])
    assert not c['sim']['available'] and not c['controls']['gui']['available']
    m = robotcaps.mapping(['a', 'b'], ['a', 'b', 'grip'], 'grip')
    assert m['missing'] == ['grip'] and m['gripper'] is False


def test_custom_robot_urdf_limits_shape_the_simulated_follower(world, tmp_path):
    rdir = tmp_path / 'robots'
    joints = [{'name': 'base', 'type': 'revolute', 'lower': -1.0, 'upper': 1.0},
              {'name': 'lift', 'type': 'revolute', 'lower': -0.5, 'upper': 2.0},
              {'name': 'slide', 'type': 'prismatic', 'lower': 0.0, 'upper': 0.04},
              {'name': 'fixed_tip', 'type': 'fixed'}]
    write_custom(rdir, 'My-Arm 2', joints, {'joints': [
        {'urdf_joint': 'base', 'ros_joint': 'base_joint'},
        {'urdf_joint': 'lift', 'ros_joint': 'lift_joint', 'soft_lower': -0.4, 'soft_upper': 1.5},
        {'urdf_joint': 'slide', 'ros_joint': 'gripper_slide'}]})
    h = Hardware(tmp_path / 'hw', Environments(), monitor=False, robots_dir=rdir)
    h.ros = FakeRos()
    try:
        st = h.setup('my_arm_2', mode='simulation', topics={'follower': '/joint_states'})
        r = st['planning']['robot']
        assert r['names'] == ['base_joint', 'lift_joint', 'gripper_slide']
        assert r['gripper'] == 'gripper_slide' and r['dof'] == 3
        assert r['limits']['lift_joint'] == [-0.4, 1.5]            # soft limits win
        assert st['sim']['names'] == r['names']
        assert st['sim']['limits'][2] == [0.0, 0.04]
        st = h.sim_command('my_arm_2', {'command': 'targets', 'targets': {'lift_joint': 9}})
        assert st['sim']['target'][1] == pytest.approx(1.5)
        assert {c['id'] for c in st['planning']['checks']} >= {'robot'}
        assert opts(st, 'action_sources')['gamepad']['available']
    finally:
        h.stop()


def test_leader_missing_a_joint_blocks_recording(world, hw):
    register(world, hw, ('leader',))
    hw.setup(RT, mode='leader_sim')
    hw.ros.publish('/leader/joint_trajectory', J[:5], [0.1] * 5)   # no gripper on the leader
    st = hw.status(RT)
    ch = {c['id']: c for c in st['planning']['checks']}
    assert ch['mapping']['state'] == 'err' and 'gripper' in ch['mapping']['detail']
    assert ch['gripper']['state'] == 'err'
    assert not st['planning']['record']['available']
    assert 'no joint gripper' in st['planning']['record']['reason']


def test_only_a_leader_in_leader_follower_uses_the_simulated_follower(world, hw):
    """No follower was ever set up: the simulated follower is the target by itself. A follower
    that is set up but unplugged keeps Leader + Follower (and says why)."""
    register(world, hw, ('leader',))
    live(hw, follower=False)
    st = hw.status(RT)
    assert st['mode'] == 'leader_follower'
    assert st['planning']['config'] == 'leader_sim' and st['planning']['auto']
    assert st['planning']['selection']['target'] == 'sim' and st['sim']['running']
    world.plug('ttyACM1', serial='FOLL-B')
    s2 = hw.detect(RT)
    hw.assign(RT, [u for u in s2['unassigned'] if u['port']['device'] == world.device(
        'ttyACM1')][0]['key'], 'follower')
    world.unplug('ttyACM1')
    hw.ports_at = 0
    st = hw.status(RT)
    assert st['planning']['config'] != 'leader_sim' and not st['planning']['auto']


# ------------------------------------------------ follower without bridge.py ---
def test_detect_reports_why_a_port_was_not_scanned(world, hw):
    world.plug('ttyACM0', serial='FOLL-B')
    world.hold('ttyACM0')                    # e.g. a bridge.py or another program
    st = hw.detect(RT)
    u = st['unassigned'][0]
    assert u['probe'] and u['probe']['skipped'] and 'in use by' in u['probe']['error']


def test_serial_errors_are_explained():
    from physios_trainer.hardware.manager import explain_serial_error
    e = explain_serial_error("[Errno 16] Device or resource busy: '/dev/ttyACM0'", '/dev/ttyACM0')
    assert 'ModemManager' in e and 'Errno 16' in e
    assert 'dialout' in explain_serial_error('[Errno 13] Permission denied', '/dev/ttyACM0')
    assert 'power supply' in explain_serial_error('[TxRxResult] There is no status packet!', 'x')
    assert explain_serial_error('something else', 'x') == 'something else'


def test_follower_read_over_serial_is_published_for_ros(world, hw):
    """Follower Only without bridge.py: RawBot Studio's read-only session reads the motors and
    publishes their joint states on the follower topic; hand-guided recording works; direct
    control (which needs bridge.py to move the arm) stays off."""
    world.plug('ttyACM1', serial='FOLL-B')
    st = hw.detect(RT)
    hw.assign(RT, st['unassigned'][0]['key'], 'follower')
    hw.setup(RT, mode='follower_only')
    st = hw.connect(RT, 'follower')
    for _ in range(40):
        if hw._monitor_status(RT, 'follower').get('phase') == 'running':
            break
        time.sleep(0.1)
    world.set('ttyACM1', actual={'1': 3072, '2': 2048, '3': 2048, '4': 2048, '5': 2048,
                                 '6': 2048})
    time.sleep(0.4)
    hw.report_action_source(RT, True, 'follower')
    st = hw.status(RT)
    F = st['arms']['follower']
    assert F['state'] == 'connected' and F['source'] == 'serial'
    assert F['publishes'] == '/joint_states' and hw.relay.owns('/joint_states')
    msg = hw.relay.tick()
    assert msg['name'][0] == 'shoulder_pan' and msg['position'][0] == pytest.approx(math.pi / 2)
    assert st['capabilities']['hand_guided_record']['available']
    assert not st['capabilities']['follower_control']['available']      # needs bridge.py
    p = st['planning']
    assert p['selection']['action'] == 'hand_guided' and p['record']['available'], p['record']
    # our own messages do not make the topic look like a bridge.py
    hw.ros.publish('/joint_states', J, [0.0] * 6, publisher=None)
    assert hw._ros_live(RT, 'follower', hw.reg.setup(RT))[0] is False
    # bridge.py starts publishing: RawBot Studio stops relaying
    hw.ros.pubs['/joint_states'] = ['/so101_bridge']
    st = hw.status(RT)
    assert 'publishes' not in st['arms']['follower'] and not hw.relay.topic
