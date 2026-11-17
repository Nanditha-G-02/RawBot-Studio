# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""What the Planning and Record sessions can use with the hardware that is there now.

Three independent choices, each option available or not with a short reason:

    action source   where the demonstrated action comes from (leader arm, hand guiding,
                    GUI / keyboard / gamepad, a script, a recorded trajectory)
    target robot    what moves: the physical follower or the simulated robot
    observation     whose joint states are recorded as the robot state

and, for the current selection, whether a demonstration can be recorded and how
physical_ai_server must take the action (``action_param``):

    leader    action = the command topic (the leader arm, or RawBot Studio's supervised
              direct control publishing there)
    follower  action = the measured joints of the recorded robot (hand guiding, or the
              simulated robot driven by GUI / keyboard / gamepad / script / replay)

The recorded state always comes from the follower state topic of the robot configuration:
the physical follower, or - when the configuration has no physical follower - the
simulated robot publishing that topic. The dataset format does not change.
"""

from physios_trainer.hardware import capabilities as capmod
from physios_trainer.hardware import robotcaps
from physios_trainer.hardware.registry import MODES

CONFIGS = {
    'leader_follower': 'Leader + Follower',
    'leader_only': 'Leader Only',
    'leader_sim': 'Physical Leader + Simulated Follower',
    'sim_leader': 'Simulated Leader + Physical Follower',
    'follower_only': 'Follower Only',
    'simulation': 'Simulation Only',
    'physical_sim': 'Physical + Simulated',
}
ACTIONS = [
    ('leader', 'Leader arm'),
    ('hand_guided', 'Hand guiding'),
    ('gui', 'GUI joint control'),
    ('keyboard', 'Keyboard'),
    ('gamepad', 'Gamepad'),
    ('script', 'Scripted movement'),
    ('replay', 'Trajectory replay'),
]
TARGETS = [('follower', 'Physical follower'), ('sim', 'Simulated robot')]
OBSERVATIONS = [('follower', 'Follower joint states'), ('sim', 'Simulated joint states')]
MANUAL = ('gui', 'keyboard', 'gamepad')
# inputs that move the simulated robot; with the physical follower as the target the simulated
# robot is the leader and the follower follows it (virtual teleoperation, jog_worker follow mode)
SIM_LEADS = MANUAL + ('script', 'replay')
DEFAULT_ORDER = {
    'leader_follower': ('leader', 'follower', 'follower'),
    'leader_only': ('leader', 'sim', 'sim'),
    'leader_sim': ('leader', 'sim', 'sim'),
    'follower_only': ('hand_guided', 'follower', 'follower'),
    'simulation': ('gui', 'sim', 'sim'),
}


def _opt(key, label, reason='', short=''):
    return {'id': key, 'label': label, 'available': not reason, 'reason': reason,
            'short': '' if not reason else (short or 'not available')}


def options(mode, arms, caps, sim, selection=None, cameras=None, extra=None, robot=None,
            auto=False):
    """mode: hardware configuration; arms: arm cards; caps: capabilities.compute();
    sim: SimRobot.view() plus {'can_own_state': bool, 'own_reason': str};
    selection: stored {action, target, observation}; cameras: [{name, topic, live}];
    extra: {'jog_running': bool, 'compare': {...}|None}; robot: robotcaps.capabilities()
    (joints, gripper, simulation model, control methods); auto: only a physical leader is
    there, so the simulated follower was chosen as the target by itself."""
    extra = extra or {}
    robot = robot or robotcaps.capabilities('', ['?'])     # unknown robot: nothing gated
    rctl = robot.get('controls') or {}
    used = MODES.get(mode, MODES['leader_follower'])['arms']
    L, F = arms.get('leader') or {}, arms.get('follower') or {}
    lr = L.get('state') == 'connected' and L.get('source') == 'ros'
    # the leader read by RawBot Studio's own serial session drives the simulated robot too
    ls = L.get('state') == 'connected' and L.get('source') == 'serial' and 'leader' in used
    fr = F.get('state') == 'connected' and F.get('source') == 'ros'
    # follower joint states on ROS: bridge.py, or RawBot Studio relaying its serial session
    fs = fr or bool(extra.get('follower_relay'))
    sim_on = bool(sim.get('running'))
    config = mode if mode in ('simulation', 'leader_sim') else (
        'physical_sim' if sim_on else mode)

    def arm_reason(role, what):
        card = L if role == 'leader' else F
        if role not in used:
            return ('%s needs the %s arm (the configuration is %s).'
                    % (what, role, MODES[mode]['label']), 'no %s' % role)
        if card.get('state') == 'connected' and card.get('source') == 'serial':
            return ('%s goes through bridge.py: disconnect the read-only session on Robots → '
                    'Hardware and start bridge.py.' % what, 'read-only session')
        return (capmod.arm_reason(mode, card, role), 'not connected')

    sim_reason = ('Needs the simulated robot: turn on Simulated twin.', 'simulation off')

    # ---- target availability per action
    def target_ok(action, target):
        c = rctl.get(action)
        if c and not c['available']:          # the robot itself does not allow this control
            return (c['reason'], c['short'])
        if target == 'sim' and not robot['sim']['available']:
            return (robot['sim']['reason'], robot['sim']['short'])
        if target == 'sim':
            if not sim_on:
                return sim_reason
            if action == 'leader' and not (lr or ls):
                return arm_reason('leader', 'The leader arm')
            if action == 'hand_guided':
                return ('Hand guiding moves the physical follower.', 'physical only')
            return None
        # physical follower
        if 'follower' not in used:
            if mode == 'leader_sim':
                return ('This configuration has no physical follower: the leader drives the '
                        'simulated follower.', 'simulated follower')
            return arm_reason('follower', 'The physical follower')
        if action == 'leader':
            c = caps.get('teleoperation') or {}
            return None if c.get('available') else (c.get('reason') or 'Needs Leader + Follower.',
                                                      'needs Leader + Follower')
        if action == 'hand_guided':
            c = caps.get('hand_guided_record') or {}
            return None if c.get('available') else (c.get('reason') or '', 'needs Follower Only')
        if action in SIM_LEADS and not robot['sim']['available']:
            return (robot['sim']['reason'], robot['sim']['short'])
        c = caps.get('follower_control') or {}
        if c.get('available') or (extra.get('jog_running') and fr):
            return None
        return (c.get('reason') or 'Direct control is not available.', 'not available')

    pref = DEFAULT_ORDER.get(mode, DEFAULT_ORDER['leader_follower'])
    t_order = [pref[1]] + [t for t, _ in TARGETS if t != pref[1]]
    actions = []
    for key, label in ACTIONS:
        order = ['sim', 'follower'] if key in ('script', 'replay') else t_order
        reasons = [target_ok(key, t) for t in order]
        ok = [r for r in reasons if r is None]
        if ok:
            actions.append(_opt(key, label))
        else:
            first = next((r for r in reasons if r and r[0]), ('Not available.', ''))
            if key == 'leader' and not lr and not ls:
                first = arm_reason('leader', 'The leader arm')
            actions.append(_opt(key, label, *first))
    by_id = {a['id']: a for a in actions}

    sel = dict(selection or {})
    # a stored choice that no longer fits falls back to the configuration's default (shown,
    # with its reason, even when that is not available yet) - never to a surprise source
    if not by_id.get(sel.get('action'), {}).get('available'):
        sel['action'] = pref[0]
    a = sel['action']
    targets = [_opt(t, lbl, *(target_ok(a, t) or ('', ''))) for t, lbl in TARGETS]
    t_ok = {t['id']: t['available'] for t in targets}
    if not t_ok.get(sel.get('target')):
        sel['target'] = next((t for t in (pref[1], 'follower', 'sim') if t_ok.get(t)), pref[1])
    tg = sel['target']

    def obs_ok(o):
        if o == 'follower':
            if not fs:
                return arm_reason('follower', 'Follower joint states')
            if tg == 'sim' and not (a == 'leader' and mode == 'leader_follower'):
                return ('The simulated robot is the target; the physical follower does not '
                        'follow this source.', 'follower not driven')
            return None
        if not sim_on:
            return sim_reason
        if tg == 'follower':
            return ('The physical follower is the target: its joint states are recorded.',
                    'follower is the target')
        if not sim.get('can_own_state'):
            return (sim.get('own_reason') or 'The simulated robot cannot publish the state.',
                    'physical follower present')
        return None
    observations = [_opt(o, lbl, *(obs_ok(o) or ('', ''))) for o, lbl in OBSERVATIONS]
    o_ok = {o['id']: o['available'] for o in observations}
    if not o_ok.get(sel.get('observation')):
        sel['observation'] = next((o for o in (pref[2], 'follower', 'sim') if o_ok.get(o)),
                                  pref[2])
    ob = sel['observation']

    # ---- recording for the selection
    reason = ''
    if not by_id[a]['available']:
        reason = by_id[a]['reason']
    elif not t_ok.get(tg):
        reason = next(t['reason'] for t in targets if t['id'] == tg)
    elif not o_ok.get(ob):
        reason = next(o['reason'] for o in observations if o['id'] == ob)
    if not reason and tg == 'follower' and a == 'leader':
        c = caps.get('record') or {}
        reason = '' if c.get('available') else c.get('reason') or 'Recording is not available.'
    if not reason and tg == 'follower' and a in SIM_LEADS and not extra.get('jog_running'):
        reason = 'Start following first: the arm follows the simulated leader.'
    if not reason and ob == 'sim' and not sim.get('state_topic'):
        reason = 'The simulated robot is not publishing the follower state topic yet.'
    if not reason and extra.get('calibration_active'):
        reason = 'A calibration is in progress (motor torque is off).'
    missing = [c for c in (cameras or []) if c.get('live') is False]
    if not reason and missing:
        reason = ('Camera %s is not publishing (%s). Recording needs every camera of the robot '
                  'configuration.' % (missing[0]['name'], missing[0]['topic']))
    action_param = 'follower' if (a == 'hand_guided' or (tg == 'sim' and a != 'leader')) \
        else 'leader'
    if a == 'leader' and tg == 'sim' and not lr and ls:
        # RawBot Studio publishes the leader joints itself (simbot.LEADER_STATE_TOPIC)
        action_param = 'leader_state'
        li = sim.get('leader_in') or {}
        if not reason and sim_on and li and li.get('stale'):
            reason = 'No fresh joint readings from the leader arm (serial session).'
        if not reason and extra.get('leader_state_supported') is False:
            reason = ('The robot runtime cannot record the leader read by RawBot Studio yet. '
                      'Rebuild it: ./docker/physios-docker.sh build robot')
    streams = [
        {'id': 'state', 'label': 'Joint positions (state)', 'from': ob},
        {'id': 'action', 'label': 'Joint actions (leader arm joint positions)'
         if a == 'leader' else 'Joint actions', 'from': a if action_param != 'follower'
         else ob},
        {'id': 'gripper', 'label': 'Gripper (state and action)', 'from': ob},
        {'id': 'cameras', 'label': '%d camera%s' % (len(cameras or []),
                                                    '' if len(cameras or []) == 1 else 's'),
         'from': 'cameras'},
        {'id': 'timestamps', 'label': 'Timestamps (fixed fps)', 'from': 'recorder'},
        {'id': 'task', 'label': 'Task instruction and plan episode', 'from': 'plan'},
        {'id': 'velocity', 'label': 'Joint velocities - derived from positions at the fixed '
         'fps (no separate column, so the dataset format stays the same)', 'from': 'derived'},
    ]

    robots = [
        {'id': 'leader', 'kind': 'physical', 'label': 'Leader arm', 'used': 'leader' in used,
         'state': L.get('state'), 'source': L.get('source'), 'joints': _joints(L)},
        {'id': 'follower', 'kind': 'physical', 'label': 'Follower arm',
         'used': 'follower' in used, 'state': F.get('state'), 'source': F.get('source'),
         'joints': _joints(F)},
        {'id': 'sim', 'kind': 'simulated', 'label': 'Simulated follower'
         if (mode == 'leader_sim' or sim.get('state_topic')) else 'Simulated robot',
         'used': sim_on,
         'state': 'running' if sim_on else 'off', 'source': 'sim',
         'joints': list(sim.get('names') or []) if sim_on else []},
    ]
    # ---- checks before starting: robot, joint mapping, limits, cameras, runtime
    def check(cid, label, ok, detail='', warn=False):
        return {'id': cid, 'label': label, 'state': 'ok' if ok else ('warn' if warn else 'err'),
                'detail': detail}
    checks = [check('robot', '%d joints%s' % (robot['dof'], ' · gripper' if robot['gripper']
                                               else ''), robot['dof'] > 0,
                    robot['source'] or 'joints unknown')]
    pairs = extra.get('pairs') or {}
    if a == 'leader' or (a in SIM_LEADS and tg == 'follower'):
        li = sim.get('leader_in') or {}
        src = (li.get('names') or list((L.get('joints') or {}).keys())) if a == 'leader' \
            else list(sim.get('names') or [])
        tgt = robot['names'] if tg == 'sim' else list((F.get('joints') or {}).keys())
        if src and tgt:
            m = robotcaps.mapping(src, tgt, robot['gripper'], pairs)
            checks.append(check('mapping', 'Mapping %d/%d' % (m['matched'], m['total']),
                                not m['missing'], 'missing on the leader: %s' % ', '.join(
                                    m['missing']) if m['missing'] else 'by joint name'))
            if robot['gripper']:
                checks.append(check('gripper', 'Gripper', m['gripper'],
                                    robot['gripper'] if m['gripper'] else
                                    'the leader has no %s' % robot['gripper']))
            if m['missing'] and not reason:
                reason = 'The %s has no joint %s of the %s.' % (
                    'leader' if a == 'leader' else 'simulated leader',
                    ', '.join(m['missing']), 'simulated follower' if tg == 'sim' else 'follower')
        if tg == 'sim' and li:
            checks.append(check('limits', 'Limits', not li.get('at_limit'),
                                'at limit: %s' % ', '.join(li['at_limit']) if li.get('at_limit')
                                else 'within limits', warn=True))
    if cameras:
        live = [c for c in cameras if c.get('live') is not False]
        checks.append(check('cameras', 'Cameras %d/%d' % (len(live), len(cameras)),
                            len(live) == len(cameras), ', '.join(c['name'] for c in cameras)))
    if action_param == 'leader_state':
        checks.append(check('runtime', 'Runtime', extra.get('leader_state_supported') is not
                            False, 'robot image supports leader_state'
                            if extra.get('leader_state_supported') else 'rebuild the robot image'
                            if extra.get('leader_state_supported') is False else 'not checked yet',
                            warn=extra.get('leader_state_supported') is None))

    if tg == 'follower' and a in SIM_LEADS and sim_on:
        config = 'sim_leader'        # the simulated robot leads the physical follower
    return {
        'config': config, 'config_label': CONFIGS[config], 'auto': bool(auto), 'robot': robot,
        'checks': checks, 'robots': robots,
        'action_sources': actions, 'targets': targets, 'observations': observations,
        'cameras': list(cameras or []), 'selection': sel,
        'record': {'available': not reason, 'reason': reason, 'action_param': action_param,
                   'streams': streams},
        'compare': extra.get('compare'),
    }


def sim_drive(sel):
    """What drives the simulated robot for a selection; None = the operator (manual, script,
    replay)."""
    if sel.get('target') == 'follower':
        if sel.get('action') in SIM_LEADS:
            # the simulated robot is the leader of the physical follower: the operator drives
            # it (a script or replay keeps running until another input is chosen)
            return 'manual' if sel.get('action') in MANUAL else None
        return 'follower'          # physical-to-simulation mirror
    if sel.get('action') == 'leader':
        return 'leader'
    if sel.get('action') in MANUAL:
        return 'manual'            # a running script or replay stops
    return None


def _joints(card):
    j = card.get('joints') or {}
    return list(j) if isinstance(j, (list, dict)) else []
