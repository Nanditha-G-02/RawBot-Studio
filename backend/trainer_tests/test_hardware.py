# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Flexible robot setup: the nine scenarios of the feature plus identity and safety rules.

Runs against a fake USB world (hw_fixtures.HwWorld: sysfs, /dev, simulated Feetech buses) and
a fake ROS watch. The real hw_worker.py runs in a subprocess, with the simulated bus.
"""

import time

import pytest

from hw_fixtures import FakeRos, HwWorld
from physios_trainer.environment import Environments
from physios_trainer.hardware.manager import Hardware, HardwareError

RT = 's101_dataset'
TOPICS = {'follower': '/joint_states', 'leader': '/leader/joint_trajectory'}
J = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


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
    h.setup(RT, topics=TOPICS)
    yield h
    h.stop()


def caps(st):
    return {k: v['available'] for k, v in st['capabilities'].items()}


def register_both(world, hw):
    world.plug('ttyACM0', serial='LEAD-A')
    world.plug('ttyACM1', serial='FOLL-B')
    st = hw.detect(RT)
    keys = {u['port']['device']: u['key'] for u in st['unassigned']}
    hw.assign(RT, keys[world.device('ttyACM0')], 'leader')
    return hw.assign(RT, keys[world.device('ttyACM1')], 'follower')


def hold(world, hw, name):
    world.hold(name)
    hw.ports_at = 0   # status() caches the port list for up to 0.3 s


def bridge_publishes(hw, leader=True, follower=True):
    if follower:
        hw.ros.publish('/joint_states', J, [0.1] * 6)
    if leader:
        hw.ros.publish('/leader/joint_trajectory', J, [0.2] * 6)


# ---------------------------------------------------------------- 1. Leader + Follower ---
def test_leader_follower_via_bridge(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM0')
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw)
    st = hw.status(RT)
    for r in ('leader', 'follower'):
        assert st['arms'][r]['state'] == 'connected'
        assert st['arms'][r]['source'] == 'ros'
    c = caps(st)
    assert c['teleoperation'] and c['record'] and c['collect']
    # bridge.py teleoperation publishes the command topic: no jog or rollout on top of it
    assert not c['follower_control'] and 'teleoperation' in st['capabilities']['follower_control']['reason']
    assert not c['policy_rollout']
    assert st['recording_gate']['available']
    # bridge.py owns the ports: no calibration from RawBot Studio, with the reason
    assert not c['leader_calibration']
    assert 'bridge.py' in st['capabilities']['leader_calibration']['reason']


# ---------------------------------------------------------------------- 2. Leader only ---
def test_leader_only(world, hw):
    world.plug('ttyACM0', serial='LEAD-A')
    st = hw.detect(RT)
    hw.assign(RT, st['unassigned'][0]['key'], 'leader')
    hw.setup(RT, mode='leader_only')
    st = hw.connect(RT, 'leader')
    L = st['arms']['leader']
    assert L['state'] == 'connected' and L['source'] == 'serial'
    assert all(j['comm'] == 'ok' for j in L['joints'].values()) and len(L['joints']) == 6
    assert st['arms']['follower']['state'] == 'unused'
    c = caps(st)
    assert c['leader_monitor'] and c['leader_capture'] and c['leader_calibration']
    # only a physical leader: its simulated follower is the target by itself, and records
    assert st['planning']['config'] == 'leader_sim' and st['planning']['auto']
    assert c['record'], st['capabilities']['record']['reason']
    assert not c['teleoperation']
    assert not c['follower_control']
    with pytest.raises(HardwareError):
        hw.connect(RT, 'follower')   # not part of Leader Only


# -------------------------------------------------------------------- 3. Follower only ---
def test_follower_only(world, hw):
    world.plug('ttyACM1', serial='FOLL-B')
    st = hw.detect(RT)
    hw.assign(RT, st['unassigned'][0]['key'], 'follower')
    hw.setup(RT, mode='follower_only')
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw, leader=False)
    hw.report_action_source(RT, True)
    st = hw.status(RT)
    c = caps(st)
    assert st['arms']['follower']['state'] == 'connected'
    assert c['follower_control'] and c['policy_rollout'] and c['follower_monitor']
    assert c['hand_guided_record'] and st['recording_gate']['available']
    assert not c['record'] and not c['teleoperation']
    # an old robot runtime without the action_source parameter: explained, not an error
    hw.report_action_source(RT, False)
    st = hw.status(RT)
    assert not st['recording_gate']['available']
    assert 'Rebuild' in st['recording_gate']['reason']


# ------------------------------------------------------------------ 4. Simulation only ---
def test_simulation_only(world, hw):
    register_both(world, hw)
    st = hw.setup(RT, mode='simulation')
    c = caps(st)
    assert c['simulation'] and c['model_config']
    hw_keys = [k for k in c if k not in ('simulation', 'model_config')]
    assert not any(c[k] for k in hw_keys)
    assert all('Simulation Only' in st['capabilities'][k]['reason'] for k in hw_keys)
    assert hw.ros.topics == {}   # nothing watched
    assert st['arms']['leader']['state'] == 'unused'


# ----------------------------------------------- 5. Leader disconnection during operation ---
def test_leader_lost_during_operation(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM0')
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw)
    hw._tick(RT)   # first tick: baseline
    hw._tick(RT)
    assert hw.status(RT)['capabilities']['record']['available']
    hw.ros.msgs['/leader/joint_trajectory'] = (time.time() - 3, J, [0] * 6)   # leader silent
    hw.ros.pubs['/leader/joint_trajectory'] = []      # and nobody publishes it any more
    hw._tick(RT)
    st = hw.status(RT)
    assert st['arms']['leader']['state'] == 'lost'
    assert st['arms']['follower']['state'] == 'connected'
    assert not st['capabilities']['record']['available']
    assert 'stopped responding' in st['capabilities']['record']['reason']
    assert st['capabilities']['follower_control']['available']   # follower features stay
    assert any(e['kind'] == 'lost' and 'Leader' in e['text'] for e in st['events'])


# --------------------------------------------- 6. Follower disconnection during operation ---
def test_follower_unplugged_during_read_only_session(world, hw):
    register_both(world, hw)
    hw.setup(RT, mode='follower_only')
    hw.connect(RT, 'follower')
    hw._tick(RT)
    world.unplug('ttyACM1')
    t0 = time.time()
    while time.time() - t0 < 5 and hw.status(RT)['arms']['follower']['state'] != 'lost':
        time.sleep(0.2)
    hw._tick(RT)
    st = hw.status(RT)
    assert st['arms']['follower']['state'] in ('lost', 'not_found')
    assert not st['capabilities']['follower_monitor']['available']
    kinds = [e['kind'] for e in st['events']]
    assert 'unplug' in kinds


# ------------------------------------------------------------------ 7. Reconnection ---
def test_reconnects_by_identity_on_a_new_port(world, hw):
    register_both(world, hw)
    hw.setup(RT, mode='follower_only')
    hw.connect(RT, 'follower')
    hw._tick(RT)
    world.unplug('ttyACM1')
    time.sleep(1.6)
    hw._tick(RT)
    # same adapter comes back under another name: identity follows the serial number
    world.plug('ttyACM7', serial='FOLL-B')
    hw._tick(RT)
    t0 = time.time()
    while time.time() - t0 < 5:
        st = hw.status(RT)
        if st['arms']['follower']['state'] == 'connected':
            break
        time.sleep(0.2)
    F = st['arms']['follower']
    assert F['state'] == 'connected' and F['port']['device'] == world.device('ttyACM7')
    assert any(e['kind'] == 'reconnect' for e in st['events'])
    assert st['arms']['leader']['state'] == 'unused'


def test_ros_arm_comes_back(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM0')
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw)
    hw._tick(RT)
    hw._tick(RT)
    hw.ros.msgs['/joint_states'] = (time.time() - 3, J, [0] * 6)
    hw._tick(RT)
    assert hw.status(RT)['arms']['follower']['state'] == 'lost'
    bridge_publishes(hw)
    hw._tick(RT)
    st = hw.status(RT)
    assert st['arms']['follower']['state'] == 'connected'
    assert any(e['kind'] == 'reconnect' for e in st['events'])


# ------------------------------------------------------------ 8. Independent calibration ---
def _wait_phase(hw, want, timeout=6):
    t0 = time.time()
    v = None
    while time.time() - t0 < timeout:
        v = hw._calib_view()
        if v and v['phase'] in want:
            return v
        time.sleep(0.05)
    raise AssertionError('calibration phase %s, wanted %s' % (v and v['phase'], want))


def test_independent_calibration_and_profile(world, hw):
    register_both(world, hw)
    world.set('ttyACM0', sweep_on_calibrate=True)
    hw.connect(RT, 'follower')   # the follower keeps its own read-only session
    before_f = world.state('ttyACM1').get('calibration')
    v = hw.calibration_start(RT, 'leader')
    _wait_phase(hw, ('ready',))
    v = hw.calibration_command(v['id'], 'begin')
    assert v['phase'] == 'torque_off'
    assert all(t == 0 for t in world.state('ttyACM0')['torque'].values())
    hw.calibration_command(v['id'], 'set_middle')
    _wait_phase(hw, ('recording',))
    time.sleep(2.5)   # simulated joints sweep through their range
    v = hw.calibration_command(v['id'], 'finish', name='Leader bench')
    v = _wait_phase(hw, ('done',))
    assert v['saved_profile']
    prof = hw.reg.profile(v['saved_profile'])
    assert prof['role'] == 'leader' and prof['name'] == 'Leader bench'
    cal = world.state('ttyACM0')['calibration']
    assert cal['5'] == {'homing_offset': prof['motors']['wrist_roll']['homing_offset'],
                        'range_min': 0, 'range_max': 4095}
    assert cal['1']['range_max'] - cal['1']['range_min'] > 100
    # the follower was not touched and stayed connected
    assert world.state('ttyACM1').get('calibration') == before_f
    st = hw.status(RT)
    assert st['arms']['follower']['state'] == 'connected'
    hw.calibration_dismiss()
    st = hw.detect(RT)
    assert 'Leader bench' in st['arms']['leader']['calibration']['text']


def test_cancelled_calibration_restores_motors(world, hw):
    register_both(world, hw)
    world.set('ttyACM1', calibration={str(i): {'homing_offset': 10 + i, 'range_min': 500,
                                               'range_max': 3500} for i in range(1, 7)})
    original = world.state('ttyACM1')['calibration']
    v = hw.calibration_start(RT, 'follower')
    _wait_phase(hw, ('ready',))
    hw.calibration_command(v['id'], 'begin')
    hw.calibration_command(v['id'], 'set_middle')
    _wait_phase(hw, ('recording',))
    assert world.state('ttyACM1')['calibration'] != original   # homing was written
    v = hw.calibration_command(v['id'], 'cancel')
    v = _wait_phase(hw, ('cancelled',))
    assert v['restored'] is True
    assert world.state('ttyACM1')['calibration'] == original


def test_calibration_refused_while_bridge_holds_port(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM0')
    st = hw.status(RT)
    assert not st['arms']['leader']['can_calibrate']
    assert 'bridge.py' in st['arms']['leader']['calibrate_reason']
    with pytest.raises(HardwareError):
        hw.calibration_start(RT, 'leader')


# ------------------------------------------------------ 9. Dynamic feature availability ---
def test_leader_added_later_enables_leader_follower(world, hw):
    register_both(world, hw)
    hw.setup(RT, mode='follower_only')   # both arms are plugged in when this is chosen
    hw._tick(RT)
    hw._tick(RT)
    assert hw.status(RT)['mode'] == 'follower_only'   # an arm that was already there
    world.unplug('ttyACM0')
    hw._tick(RT)
    assert hw.status(RT)['mode'] == 'follower_only'
    world.plug('ttyACM0', serial='LEAD-A')            # the leader arrives (validated scan)
    hw._tick(RT)
    st = hw.status(RT)
    assert st['mode'] == 'leader_follower' and st['mode_source'] == 'auto'
    assert any('Leader + Follower is now active' in e['text'] for e in st['events'])


def test_leader_via_ros_later_enables_leader_follower(world, hw):
    register_both(world, hw)
    hw.setup(RT, mode='follower_only')
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw, leader=False)
    hw._tick(RT)
    hw._tick(RT)
    assert hw.status(RT)['mode'] == 'follower_only'
    bridge_publishes(hw, follower=False)   # bridge.py now publishes a leader too
    hw._tick(RT)
    st = hw.status(RT)
    assert st['mode'] == 'leader_follower'
    assert st['capabilities']['record']['available'] is False or \
        st['arms']['leader']['state'] == 'connected'


def test_arm_that_appeared_just_before_the_choice_does_not_upgrade(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM0')
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw, follower=False)
    hw._tick(RT)
    hw._tick(RT)
    bridge_publishes(hw)          # the follower comes back on ROS ...
    hw.setup(RT, mode='leader_only')   # ... and the operator chooses Leader Only before a tick
    hw._tick(RT)
    hw._tick(RT)
    assert hw.status(RT)['mode'] == 'leader_only'


def test_simulation_is_never_changed_automatically(world, hw):
    register_both(world, hw)
    hw.setup(RT, mode='simulation')
    hw._tick(RT)
    bridge_publishes(hw)
    hw._tick(RT)
    assert hw.status(RT)['mode'] == 'simulation'


# ---------------------------------------------------------- identity and safety rules ---
def test_role_follows_identity_not_port(world, hw):
    register_both(world, hw)
    world.unplug('ttyACM0')
    world.unplug('ttyACM1')
    world.plug('ttyACM0', serial='FOLL-B')   # cables swapped
    world.plug('ttyACM1', serial='LEAD-A')
    hw.ports_at = 0
    st = hw.status(RT)
    assert st['arms']['leader']['port']['device'] == world.device('ttyACM1')
    assert st['arms']['follower']['port']['device'] == world.device('ttyACM0')


def test_not_connected_until_verified(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM1')   # a program holds the port, but nothing arrives on ROS
    st = hw.status(RT)
    F = st['arms']['follower']
    assert F['state'] == 'in_use' and 'bridge.py' in F['detail']
    assert not st['capabilities']['follower_control']['available']
    with pytest.raises(HardwareError) as e:
        hw.connect(RT, 'follower')
    assert e.value.status == 409


def test_missing_motor_blocks_connect_and_assign(world, hw):
    world.plug('ttyACM0', serial='LEAD-A', motors=[1, 2, 3, 4, 6])
    st = hw.detect(RT)
    u = st['unassigned'][0]
    assert u['probe']['ok'] is False and u['probe']['missing_ids'] == [5]
    with pytest.raises(HardwareError):
        hw.assign(RT, u['key'], 'leader')


def test_assign_needs_validation_when_port_is_held(world, hw):
    world.plug('ttyACM0', serial='LEAD-A')
    hold(world, hw, 'ttyACM0')
    st = hw.detect(RT)
    with pytest.raises(HardwareError) as e:
        hw.assign(RT, st['unassigned'][0]['key'], 'leader')
    assert 'bridge.py' in str(e.value)


def test_own_commands_are_not_a_leader(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM1')
    bridge_publishes(hw, leader=False)
    # only rosbridge (our own jog/rollout) publishes on the leader topic
    hw.ros.publish('/leader/joint_trajectory', J, [0] * 6, publisher=None)
    hw.ros.pubs['/leader/joint_trajectory'] = []
    st = hw.status(RT)
    assert st['arms']['leader']['state'] != 'connected'


def test_jog_prepare_validates_limits(world, hw):
    register_both(world, hw)
    hold(world, hw, 'ttyACM1')
    hold(world, hw, 'ttyACM0')
    bridge_publishes(hw, leader=False)   # teleoperation is off
    lim = [{'name': n, 'min': -1.0, 'max': 1.0, 'speed': 0.2} for n in J]
    ok = hw.jog_prepare(RT, {'names': J, 'limits': lim})
    assert ok['ok'] and ok['token']
    bad = hw.jog_prepare(RT, {'names': J, 'limits': [dict(lim[0], speed=5.0)] + lim[1:]})
    assert not bad['ok'] and any('25%' in e for e in bad['errors'])
    with pytest.raises(HardwareError):
        hw.jog_start(RT, {'names': J, 'limits': lim}, ok['token'], {'area_clear': True})


def test_estop_without_sessions_is_accepted(hw):
    r = hw.estop('cli')
    assert r['ok'] and r['stopped'] == []
