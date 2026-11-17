# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Supervised follower jog ("direct robot control") over ROS.

    python -I jog_worker.py <config.json>

Same path as host rollout: reads the follower JointState topic and publishes
trajectory_msgs/JointTrajectory on the command (leader) topic of the robot configuration;
bridge.py moves the arm. It never opens a serial port.

One joint moves at a time, only while the operator holds a jog button (a dead-man: every
move request carries a timestamp and expires after 0.3 s). Before every publish:
  * the target stays inside the joint limits the operator reviewed
  * the target moves at most ``speed`` units/s and may lead the measured position by at most
    ``max_lead``; a joint that does not follow stops the jog
  * stale joint states, a missing page heartbeat or another node publishing on the command
    topic -> hold at the measured position
Stop / emergency stop / end of session -> one hold command, then exit. Killing this process
also ends all commands (the service does this on an emergency stop).

Follow mode (``cfg['follow']``, virtual teleoperation): instead of one held joint, every joint
moves towards the simulated leader's joint states (``follow.topic``, mapped by ``follow.map``
target -> source) while the page has following switched on - with the same limits, speed and
lead caps, stale-data, heartbeat and foreign-publisher holds. A stale leader holds.
"""

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rollout_worker import Bridge, read_control, traj_msg, write_status  # noqa: E402


def now():
    return time.time()


def run(cfg_path):
    cfg = json.load(open(cfg_path))
    st_path, ctl_path = cfg['status_file'], cfg['control_file']
    names = list(cfg['names'])
    lim = {j['name']: j for j in cfg['limits']}
    speed = {n: float(lim[n]['speed']) for n in names}
    lead = {n: float(lim[n].get('max_lead') or speed[n] * 0.3) for n in names}
    rate = float(cfg.get('rate_hz', 30))
    stale_s = float(cfg.get('stale_s', 0.4))
    hb_timeout = float(cfg.get('heartbeat_timeout_s', 0.6))
    move_ttl = float(cfg.get('move_ttl_s', 0.3))
    st = {'phase': 'connecting', 'pid': os.getpid(), 'names': names, 'warnings': [],
          'error': None, 'commands_sent': 0}

    def push(**kw):
        st.update(kw)
        write_status(st_path, st)

    push()
    try:
        br = Bridge(cfg['rosbridge_url'])
    except Exception as e:  # noqa: BLE001
        push(phase='failed', error='rosbridge not reachable: %s' % e)
        return 2

    def other_publishers():
        pubs = br.call('/rosapi/publishers', {'topic': cfg['command_topic']}).get(
            'publishers') or []
        return [p for p in pubs if 'rosbridge' not in p]

    try:
        others = other_publishers()
    except (TimeoutError, RuntimeError) as e:
        push(phase='failed', error='Cannot check who commands the arm (rosapi: %s).' % e)
        return 2
    if others:
        push(phase='failed', error='Another node already commands the arm on %s: %s. Stop it '
             '(teleoperation, Inference or a rollout) first.' % (cfg['command_topic'],
                                                                  ', '.join(others)))
        return 2
    br.subscribe(cfg['state_topic'], 'sensor_msgs/msg/JointState')
    follow = cfg.get('follow') or None
    if follow:
        br.subscribe(follow['topic'], 'sensor_msgs/msg/JointState')
        fmap = {n: (follow.get('map') or {}).get(n, n) for n in names}

    def leader():
        """Simulated leader positions per target joint, or None when stale / incomplete."""
        hit = br.latest.get(follow['topic'])
        if not hit or now() - hit[0] > stale_s:
            return None
        src = dict(zip(hit[1].get('name') or [], hit[1].get('position') or []))
        if not all(fmap[n] in src for n in names):
            return None
        return {n: float(src[fmap[n]]) for n in names}
    br.advertise(cfg['command_topic'], 'trajectory_msgs/msg/JointTrajectory')

    def measured():
        hit = br.latest.get(cfg['state_topic'])
        if not hit:
            return None, None
        t, msg = hit
        m = dict(zip(msg.get('name') or [], msg.get('position') or []))
        if not all(n in m for n in names):
            return t, 'missing'
        return t, {n: float(m[n]) for n in names}

    t0 = now()
    while now() - t0 < 3.0:
        t, m = measured()
        if m is not None:
            break
        time.sleep(0.05)
    t, m = measured()
    if m is None:
        push(phase='failed', error='No joint states on %s.' % cfg['state_topic'])
        return 2
    if m == 'missing':
        push(phase='failed', error='The joint states do not contain all of %s.' %
             ', '.join(names))
        return 2
    push(phase='ready', measured=m)

    target = None
    moving = None
    last_check = now()
    lag_since = None
    started = now()
    last_hb_ok = now()

    def publish(pos):
        br.publish(cfg['command_topic'], traj_msg(names, [pos[n] for n in names]))
        st['commands_sent'] += 1

    def hold(reason, m_now):
        nonlocal target, moving
        if target is not None and m_now and m_now != 'missing':
            publish(m_now)
        target, moving = None, None
        push(phase='ready', held_reason=reason, moving=None)

    period = 1.0 / rate
    try:
        while True:
            tick = now()
            ctl = read_control(ctl_path)
            t_state, m = measured()
            cmd = ctl.get('command')
            hb = float(ctl.get('heartbeat') or 0)
            if cmd in ('stop', 'estop'):
                hold('stopped', m)
                push(phase='stopped', end_reason='Emergency stop' if cmd == 'estop' else
                     'Stopped by the operator.')
                return 0
            if tick - started > float(cfg.get('max_duration_s', 900)):
                hold('time limit', m)
                push(phase='stopped', end_reason='Session time limit reached.')
                return 0
            hb_ok = hb and tick - hb <= hb_timeout
            if hb_ok:
                last_hb_ok = tick
            elif tick - last_hb_ok > 5.0:
                hold('page gone', m)
                push(phase='stopped', end_reason='The page stopped sending its heartbeat.')
                return 0
            fresh = t_state is not None and tick - t_state <= stale_s and m != 'missing'
            if tick - last_check > 2.0:
                last_check = tick
                try:
                    others = other_publishers()
                except (TimeoutError, RuntimeError):
                    others = []
                if others:
                    hold('conflict', m if fresh else None)
                    push(phase='failed', error='Another node started commanding the arm: %s.'
                         % ', '.join(others))
                    return 2
            if not fresh:
                if target is not None:
                    target, moving = None, None
                push(phase='stale' if st['phase'] != 'stale' else 'stale',
                     held_reason='no fresh joint states')
                time.sleep(period)
                continue
            if st['phase'] == 'stale':
                push(phase='ready', held_reason=None)
            if follow:
                on = bool(hb_ok and (ctl.get('follow') or {}).get('on'))
                goal = leader() if on else None
                if not on or goal is None:
                    if target is not None:
                        hold('paused' if not on else 'leader stale', m)
                    push(measured=m, following=False,
                         held_reason=None if not on else 'no fresh simulated leader')
                    time.sleep(max(0.0, period - (now() - tick)))
                    continue
                if target is None:
                    target = dict(m)
                limited = []
                for n in names:
                    lo, hi = float(lim[n]['min']), float(lim[n]['max'])
                    g = min(hi, max(lo, goal[n]))
                    if g != goal[n]:
                        limited.append(n)
                    step = max(-speed[n] * period, min(speed[n] * period, g - target[n]))
                    nxt = target[n] + step
                    if abs(nxt - m[n]) <= lead[n]:
                        target[n] = nxt
                lagging = [n for n in names if abs(target[n] - m[n]) > lead[n]]
                if lagging:
                    lag_since = lag_since or tick
                    if tick - lag_since > 1.0:
                        hold('not following', m)
                        push(phase='failed', error='%s does not follow the command. Check the '
                             'arm and bridge.py.' % ', '.join(lagging))
                        return 2
                else:
                    lag_since = None
                publish(target)
                push(phase='moving', moving='all', following=True, at_limit=limited[0] if
                     limited else None, limited=limited, measured=m, target=dict(target))
                time.sleep(max(0.0, period - (now() - tick)))
                continue
            mv = ctl.get('move') or {}
            want = mv.get('joint') if (hb_ok and mv.get('dir') in (-1, 1) and
                                       tick - float(mv.get('t') or 0) <= move_ttl) else None
            if want and want not in names:
                want = None
            if want:
                d = int(mv['dir'])
                if target is None:
                    target = dict(m)   # all other joints hold where they are
                lo, hi = float(lim[want]['min']), float(lim[want]['max'])
                nxt = target[want] + d * speed[want] * period
                # inside the limits; from outside, only towards the range
                if (d > 0 and target[want] >= hi) or (d < 0 and target[want] <= lo):
                    push(moving=want, at_limit=want)
                else:
                    nxt = min(hi, max(lo, nxt)) if lo <= target[want] <= hi else nxt
                    lag = abs(nxt - m[want])
                    if lag <= lead[want]:
                        target[want] = nxt
                        lag_since = None
                    else:
                        lag_since = lag_since or tick
                        if tick - lag_since > 1.0:
                            hold('not following', m)
                            push(phase='failed', error='%s does not follow the command (it '
                                 'lags by %.3f). Check the arm and bridge.py.' % (want, lag))
                            return 2
                    publish(target)
                    push(phase='moving', moving=want, at_limit=None, measured=m,
                         target=dict(target))
            elif target is not None:
                hold('released', m)
            else:
                push(measured=m)
            time.sleep(max(0.0, period - (now() - tick)))
    finally:
        br.close()


if __name__ == '__main__':
    sys.exit(run(sys.argv[1]))
