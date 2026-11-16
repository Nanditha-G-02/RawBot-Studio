# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""What the connected hardware allows. Every feature that is not available says why.

An arm counts only when its communication is verified: ``state == 'connected'``, either
through RawBot Studio's own read-only serial session (``source == 'serial'``) or through
live ROS messages from bridge.py (``source == 'ros'``). Recording, teleoperation and
anything that moves the arm go through bridge.py, so they need the ROS path.
"""

from physios_trainer.hardware.registry import MODES

FEATURES = [
    # key, label, group
    ('teleoperation', 'Leader–follower teleoperation', 'leader_follower'),
    ('record', 'Synchronized data recording', 'leader_follower'),
    ('collect', 'Demonstration collection (Plan → Record)', 'leader_follower'),
    ('leader_calibration', 'Leader calibration', 'leader'),
    ('leader_monitor', 'Leader joint monitoring', 'leader'),
    ('leader_capture', 'Leader data capture', 'leader'),
    ('follower_calibration', 'Follower calibration', 'follower'),
    ('follower_control', 'Direct robot control (jog)', 'follower'),
    ('follower_monitor', 'Follower joint monitoring', 'follower'),
    ('hand_guided_record', 'Hand-guided recording (follower only)', 'follower'),
    ('inference', 'Run a trained policy (Inference page)', 'follower'),
    ('policy_rollout', 'Run a trained policy (host rollout)', 'follower'),
    ('simulation', 'Simulation view and planning', 'always'),
    ('model_config', 'Robot model configuration', 'always'),
]
LABEL = {k: l for k, l, _ in FEATURES}


def arm_reason(mode, card, role):
    """Why an arm cannot be used, in the operator's words."""
    st = card.get('state')
    name = role.capitalize()
    if role not in MODES.get(mode, MODES['leader_follower'])['arms']:
        return '%s is not part of the %s configuration.' % (name, MODES[mode]['label'])
    return {
        'disconnected': '%s arm was disconnected in RawBot Studio. Press Connect.' % name,
        'lost': '%s arm stopped responding. Check its cable and power, then Reconnect.'
                % name,
        'in_use': card.get('detail') or '%s port is in use by another program.' % name,
        'not_found': '%s arm is not plugged in.' % name,
        'not_configured': 'No %s arm is set up. Use Detect Hardware.' % role,
        'error': card.get('detail') or '%s arm did not pass the motor check.' % name,
        'detected': '%s arm is detected but not connected. Press Connect.' % name,
    }.get(st, '%s arm is not connected.' % name)


def compute(mode, arms, extra=None):
    """mode: setup mode; arms: {'leader': card, 'follower': card}; extra: {
    'active_rollout': bool, 'jog_active': bool, 'calibration_active': bool,
    'action_source_supported': bool|None, 'command_topic': str|None}."""
    extra = extra or {}
    used = MODES.get(mode, MODES['leader_follower'])['arms']
    L, F = arms.get('leader') or {}, arms.get('follower') or {}
    lc, fc = L.get('state') == 'connected', F.get('state') == 'connected'
    lr, fr = lc and L.get('source') == 'ros', fc and F.get('source') == 'ros'
    out = {}

    def put(key, ok, reason=''):
        out[key] = {'label': LABEL[key], 'available': bool(ok), 'reason': '' if ok else reason}

    def why_arm(card, role):
        return arm_reason(mode, card, role)

    def why_ros(card, role, what):
        if card.get('state') == 'connected' and card.get('source') == 'serial':
            return ('%s goes through bridge.py: RawBot Studio has the %s port open read-only. '
                    'Disconnect it here and start bridge.py.' % (what, role))
        return why_arm(card, role)

    sim = mode == 'simulation'
    sim_reason = 'Simulation Only is selected - no physical robot is used.'
    lf = mode == 'leader_follower'

    if sim:
        for key, _, group in FEATURES:
            if group != 'always':
                put(key, False, sim_reason)
    else:
        # ---- both arms
        if not lf:
            r = 'Needs Leader + Follower (now %s).' % MODES[mode]['label']
            put('teleoperation', False, r)
            put('record', False, r)
            put('collect', False, r)
        else:
            r = None
            if not lr:
                r = why_ros(L, 'leader', 'Teleoperation')
            elif not fr:
                r = why_ros(F, 'follower', 'Teleoperation')
            put('teleoperation', not r, r)
            rr = None if (lr and fr) else (why_ros(L, 'leader', 'Recording') if not lr else
                                           why_ros(F, 'follower', 'Recording'))
            put('record', not rr, rr)
            put('collect', not rr, rr)
        # ---- leader
        lu = 'leader' in used
        put('leader_monitor', lu and lc, why_arm(L, 'leader'))
        put('leader_capture', lu and lc, why_arm(L, 'leader'))
        put('leader_calibration', lu and L.get('can_calibrate'),
            L.get('calibrate_reason') or why_arm(L, 'leader'))
        # ---- follower
        fu = 'follower' in used
        put('follower_monitor', fu and fc, why_arm(F, 'follower'))
        put('follower_calibration', fu and F.get('can_calibrate'),
            F.get('calibrate_reason') or why_arm(F, 'follower'))
        if not fu:
            r = why_arm(F, 'follower')
            put('follower_control', False, r)
            put('inference', False, r)
            put('policy_rollout', False, r)
        else:
            r = None if fr else why_ros(F, 'follower', 'Direct control')
            if not r and not extra.get('command_topic'):
                r = 'No command topic in the robot configuration (joint_topic_list leader entry).'
            cmd_pubs = extra.get('command_publishers') or []
            busy = ('%s already commands the arm on %s (teleoperation). Stop teleoperation '
                    'first.' % (', '.join(cmd_pubs), extra.get('command_topic'))
                    if cmd_pubs else None)
            if not r and extra.get('active_rollout'):
                r = 'A policy rollout is running.'
            if not r and busy:
                r = busy
            put('follower_control', not r, r)
            # the Inference page runs in the robot runtime: it only needs the follower on ROS
            # (the original behaviour with a leader publishing is kept)
            ir = None if fr else why_ros(F, 'follower', 'Inference')
            put('inference', not ir, ir)
            pr = None if fr else why_ros(F, 'follower', 'A policy rollout')
            if not pr and busy:
                pr = busy
            put('policy_rollout', not pr, pr)
        if mode != 'follower_only':
            put('hand_guided_record', False, 'Hand-guided recording is for Follower Only '
                '(with a leader, the leader is the action source).')
        else:
            r = None if fr else why_ros(F, 'follower', 'Recording')
            if not r and extra.get('action_source_supported') is False:
                r = ('The robot runtime is older than this feature. Rebuild it: '
                     './docker/physios-docker.sh build robot')
            put('hand_guided_record', not r, r)
    put('simulation', True)
    put('model_config', True)
    if extra.get('jog_active'):
        for k in ('teleoperation', 'record', 'collect', 'inference', 'policy_rollout',
                  'hand_guided_record'):
            if out[k]['available']:
                out[k] = dict(out[k], available=False,
                              reason='Direct control (jog) is running. Stop it first.')
    if extra.get('calibration_active'):
        for k in ('teleoperation', 'record', 'collect', 'follower_control', 'inference',
                  'policy_rollout', 'hand_guided_record'):
            if out[k]['available']:
                out[k] = dict(out[k], available=False,
                              reason='A calibration is in progress (motor torque is off).')
    return out


def recording_gate(mode, caps):
    """The capability that allows Start on the Record page for this mode."""
    return caps['hand_guided_record'] if mode == 'follower_only' else caps['record']
