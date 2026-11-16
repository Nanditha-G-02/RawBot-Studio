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
from physios_trainer.hardware.registry import MODES

CONFIGS = {
    'leader_follower': 'Leader + Follower',
    'leader_only': 'Leader Only',
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
DEFAULT_ORDER = {
    'leader_follower': ('leader', 'follower', 'follower'),
    'leader_only': ('leader', 'sim', 'sim'),
    'follower_only': ('hand_guided', 'follower', 'follower'),
    'simulation': ('gui', 'sim', 'sim'),
}


def _opt(key, label, reason='', short=''):
    return {'id': key, 'label': label, 'available': not reason, 'reason': reason,
            'short': '' if not reason else (short or 'not available')}


def options(mode, arms, caps, sim, selection=None, cameras=None, extra=None):
    """mode: hardware configuration; arms: arm cards; caps: capabilities.compute();
    sim: SimRobot.view() plus {'can_own_state': bool, 'own_reason': str};
    selection: stored {action, target, observation}; cameras: [{name, topic, live}];
    extra: {'jog_running': bool, 'compare': {...}|None}."""
    extra = extra or {}
    used = MODES.get(mode, MODES['leader_follower'])['arms']
    L, F = arms.get('leader') or {}, arms.get('follower') or {}
    lr = L.get('state') == 'connected' and L.get('source') == 'ros'
    fr = F.get('state') == 'connected' and F.get('source') == 'ros'
    sim_on = bool(sim.get('running'))
    config = 'simulation' if mode == 'simulation' else ('physical_sim' if sim_on else mode)

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
        if target == 'sim':
            if not sim_on:
                return sim_reason
            if action == 'leader' and not lr:
                return arm_reason('leader', 'The leader arm')
            if action == 'hand_guided':
                return ('Hand guiding moves the physical follower.', 'physical only')
            return None
        # physical follower
        if action in ('script', 'replay'):
            return ('Scripts and replays drive the simulated robot only (a physical arm moves '
                    'only under supervised control).', 'simulation only')
        if 'follower' not in used:
            return arm_reason('follower', 'The physical follower')
        if action == 'leader':
            c = caps.get('teleoperation') or {}
            return None if c.get('available') else (c.get('reason') or 'Needs Leader + Follower.',
                                                      'needs Leader + Follower')
        if action == 'hand_guided':
            c = caps.get('hand_guided_record') or {}
            return None if c.get('available') else (c.get('reason') or '', 'needs Follower Only')
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
            if key == 'leader' and not lr:
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
            if not fr:
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
    if not reason and tg == 'follower' and a in MANUAL and not extra.get('jog_running'):
        reason = 'Start direct control first: its commands are the recorded action.'
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
    streams = [
        {'id': 'state', 'label': 'Joint positions (state)', 'from': ob},
        {'id': 'action', 'label': 'Joint actions', 'from': a if action_param == 'leader'
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
        {'id': 'sim', 'kind': 'simulated', 'label': 'Simulated robot', 'used': sim_on,
         'state': 'running' if sim_on else 'off', 'source': 'sim',
         'joints': list(sim.get('names') or []) if sim_on else []},
    ]
    return {
        'config': config, 'config_label': CONFIGS[config], 'robots': robots,
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
        return 'follower'          # physical-to-simulation mirror
    if sel.get('action') == 'leader':
        return 'leader'
    if sel.get('action') in MANUAL:
        return 'manual'            # a running script or replay stops
    return None


def _joints(card):
    j = card.get('joints') or {}
    return list(j) if isinstance(j, (list, dict)) else []
