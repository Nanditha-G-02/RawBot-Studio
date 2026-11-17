#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""RawBot Studio hardware doctor: walks the chain from the USB cable to the joint states that
RawBot Studio receives, and names the first link that fails with its fix.

    ./scripts/hw-doctor.sh [robot_type] [--role follower|leader] [--port /dev/ttyACM0]

  1 USB serial adapter (Linux sysfs)       4 LeRobot motor scan (read-only: ping + read)
  2 permission to open it (dialout)       5 RawBot Studio training service + hardware status
  3 who holds the port (ModemManager, ...) 6 robot runtime: container, rosbridge, ROS topics

Nothing is written to the motors and nothing moves. Run it on the robot PC, with bridge.py
stopped for steps 3-4 (the scan needs the port free).
"""

import argparse
import grp
import json
import os
import pwd
import shutil
import socket
import subprocess
import sys
import tempfile
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / 'backend'))
from physios_trainer.hardware import ports as portmod  # noqa: E402
from physios_trainer.hardware.registry import motor_spec  # noqa: E402

WORKER = HERE / 'backend' / 'physios_trainer' / 'hardware' / 'hw_worker.py'
OK, WARN, FAIL = '\033[32m✓\033[0m', '\033[33m!\033[0m', '\033[31m✗\033[0m'
first_fail = []


def say(mark, text, fix=None):
    print(' %s %s' % (mark, text))
    if fix:
        for line in fix.strip().splitlines():
            print('     → %s' % line)
    if mark == FAIL and not first_fail:
        first_fail.append((text, fix))


def run(cmd, timeout=15):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or '') + (p.stderr or '')
    except (OSError, subprocess.SubprocessError) as e:
        return 127, str(e)


def service_active(name):
    if not shutil.which('systemctl'):
        return False
    return run(['systemctl', 'is-active', name])[1].strip() == 'active'


def lerobot_python():
    if os.environ.get('RAWBOT_HW_PYTHON'):
        return os.environ['RAWBOT_HW_PYTHON']
    cands = []
    if shutil.which('conda'):
        code, out = run(['conda', 'env', 'list', '--json'], 30)
        try:
            cands += [str(Path(e) / 'bin' / 'python') for e in json.loads(out)['envs']
                      if Path(e).name == 'lerobot']
        except (ValueError, KeyError):
            pass
    home = Path.home()
    for base in ('miniconda3', 'anaconda3', 'miniforge3', 'mambaforge', '.conda'):
        cands.append(str(home / base / 'envs' / 'lerobot' / 'bin' / 'python'))
    for c in cands:
        if os.path.exists(c):
            code, _ = run([c, '-c', 'import lerobot'], 60)
            if code == 0:
                return c
    return None


def http_json(url, body=None, timeout=60):
    req = urllib.request.Request(url, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(req, timeout=timeout))


def port_open(host, port):
    try:
        with socket.create_connection((host, port), timeout=1.5):
            return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('robot_type', nargs='?', default='s101_dataset')
    ap.add_argument('--role', default='follower', choices=('follower', 'leader'))
    ap.add_argument('--port', help='the arm\'s port (default: every USB serial port)')
    ap.add_argument('--trainer', default=os.environ.get('PHYSIOS_TRAINER_URL',
                                                         'http://127.0.0.1:8766'))
    ap.add_argument('--container', default='physios-robot')
    a = ap.parse_args()
    print('RawBot Studio hardware doctor · robot %s · %s arm\n' % (a.robot_type, a.role))

    # ---- 1 USB
    print('1. USB serial adapter')
    ps = [p for p in portmod.list_ports() if not a.port or p['device'] == a.port]
    if not ps:
        say(FAIL, 'No USB serial port%s.' % (' ' + a.port if a.port else ''),
            'Use a data USB cable (not charge-only) and plug it straight into the PC.\n'
            'Power the arm board, then: sudo dmesg | tail -20  (look for ttyACM / ttyUSB)\n'
            'If dmesg shows the device and then "brltty" or a disconnect: sudo apt remove brltty')
    for p in ps:
        say(OK, '%s  %s:%s %s%s' % (p['device'], p['vid'], p['pid'], p.get('product') or '',
                                     ' S/N ' + p['serial'] if p.get('serial') else
                                     ' (no serial number: identified by USB socket)'))
    if service_active('brltty'):
        say(WARN, 'brltty is running: it can claim USB serial adapters (CH340 / CH341).',
            'sudo systemctl disable --now brltty   (or sudo apt remove brltty)')

    # ---- 2 permission
    print('\n2. Permission')
    user = pwd.getpwuid(os.getuid()).pw_name
    groups = {g.gr_name for g in grp.getgrall() if user in g.gr_mem} | {
        grp.getgrgid(os.getgid()).gr_name}
    if 'dialout' not in groups and os.getuid() != 0:
        say(WARN, '%s is not in the "dialout" group.' % user,
            'sudo usermod -aG dialout %s   then log out and back in' % user)
    for p in ps:
        if p['access'] == 'denied':
            say(FAIL, 'No permission to open %s.' % p['device'],
                'sudo usermod -aG dialout %s   then log out and back in\n'
                '(quick test only: sudo chmod a+rw %s)' % (user, p['device']))
        elif p['access'] == 'ok':
            say(OK, '%s can be opened by %s.' % (p['device'], user))

    # ---- 3 holders
    print('\n3. Who holds the port')
    held = portmod.holders([p['device'] for p in ps])
    for p in ps:
        hs = held.get(p['device']) or []
        if hs:
            say(FAIL, '%s is open in %s.' % (p['device'], ', '.join(
                '%s (PID %d)' % (h['name'], h['pid']) for h in hs)),
                'Stop it for the scan (bridge.py: Ctrl+C), or keep it and let RawBot Studio\n'
                'read the arm through bridge.py\'s ROS topics (skip to step 6).')
        else:
            say(OK, 'No program of %s has %s open.' % (user, p['device']))
        p['held'] = bool(hs)
    if service_active('ModemManager'):
        say(WARN, 'ModemManager is running: it opens new /dev/ttyACM* ports for a few seconds and '
                  'can keep motor boards busy. RawBot Studio cannot see it (another user).',
            'sudo systemctl stop ModemManager   (permanently: sudo systemctl disable ModemManager)')
    if shutil.which('fuser') and ps:
        say(OK, 'For programs of every user: sudo fuser -v %s' % ' '.join(p['device'] for p in ps))

    # ---- 4 motors
    print('\n4. Motors (LeRobot, read-only)')
    spec = motor_spec(a.robot_type)
    py = lerobot_python()
    if not spec:
        say(WARN, 'No motor table for %s: this robot is read only through its ROS topics.'
            % a.robot_type)
    elif not py:
        say(FAIL, 'No Python environment with LeRobot found (conda env "lerobot").',
            'conda activate lerobot && pip install lerobot   or set RAWBOT_HW_PYTHON=/path/to/python')
    else:
        say(OK, 'LeRobot: %s' % py)
        for p in ps:
            if p.get('held') or p['access'] == 'denied':
                say(WARN, '%s skipped (see steps 2-3).' % p['device'])
                continue
            with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
                json.dump({'port': p['device'], 'motors': spec}, f)
            code, out = run([py, '-I', str(WORKER), 'probe', f.name], 40)
            os.unlink(f.name)
            line = next((x for x in out.splitlines() if x.startswith('RAWBOT_HW ')), None)
            if not line:
                say(FAIL, '%s: the scan did not run: %s' % (p['device'], out.strip()[-300:]),
                    'Errno 16 / busy: another program has the port (step 3, ModemManager).')
                continue
            r = json.loads(line[10:])
            found = sorted(int(i) for i in (r.get('found') or {}))
            if r.get('ok'):
                say(OK, '%s: motors %s answer at %s baud · calibration: %s' % (
                    p['device'], found, r.get('baudrate'), r.get('calibration_state')))
            else:
                say(FAIL, '%s: %s (answering: %s, missing: %s)' % (
                    p['device'], r.get('error'), found or 'none', r.get('missing_ids')),
                    'No motor: switch on the arm\'s own power supply (USB powers only the board).\n'
                    'Some missing: check the 3-pin cables between those motors.\n'
                    'Wrong baud rate: lerobot-setup-motors sets 1 000 000 baud.')

    # ---- 5 training service
    print('\n5. RawBot Studio training service (%s)' % a.trainer)
    st = None
    try:
        http_json(a.trainer.rstrip('/') + '/health', timeout=5)
        st = http_json(a.trainer.rstrip('/') + '/hardware/status?robot_type=' + a.robot_type,
                       timeout=30)
        say(OK, 'Running · configuration %s' % st.get('mode_label'))
    except Exception as e:  # noqa: BLE001
        say(FAIL, 'Not reachable (%s).' % e, './start_physios.sh --with-robot')
    topic = None
    if st:
        card = st['arms'][a.role]
        topic = (st.get('topics') or {}).get(a.role)
        mark = OK if card['state'] == 'connected' else WARN
        say(mark, '%s arm: %s · %s' % (a.role.capitalize(), card['state'], card.get('detail')))
        if card['state'] == 'not_configured':
            say(FAIL, 'The %s arm is not set up in RawBot Studio.' % a.role,
                'Robots → Hardware → Detect Hardware, then "Assign as %s" on the port,\n'
                'then Connect (read-only; bridge.py stopped) - or start bridge.py.' % a.role)
        if card['state'] == 'unused':
            say(WARN, 'The %s arm is not part of the configuration %s.' % (a.role,
                                                                        st.get('mode_label')))
        if card.get('publishes'):
            say(OK, 'RawBot Studio publishes its readings on %s (no bridge.py needed for '
                    'state).' % card['publishes'])
        if not st['ros']['connected'] and st['ros'].get('error'):
            say(FAIL, 'RawBot Studio cannot reach ROS (rosbridge %s): %s' % (
                st['ros']['url'], st['ros']['error']),
                'Start the robot runtime: ./start_physios.sh --with-robot  (step 6 shows why)')
        elif not st['ros']['connected']:
            say(WARN, 'RawBot Studio has not connected to ROS yet: it starts once a page has '
                      'loaded the robot configuration (topics: %s).' % (st.get('topics') or 'none'),
                'Open RawBot Studio in the browser with the robot selected on Home.')

    # ---- 6 robot runtime / ROS
    print('\n6. Robot runtime (Docker) and ROS')
    if not shutil.which('docker'):
        say(FAIL, 'docker is not installed: ROS runs in the %s container.' % a.container,
            'Install Docker (README), then ./scripts/install.sh --robot')
    else:
        code, out = run(['docker', 'ps', '-a', '--filter', 'name=^%s$' % a.container,
                         '--format', '{{.Status}}'])
        status = out.strip()
        if code != 0:
            say(FAIL, 'docker ps failed: %s' % status[:200],
                'sudo usermod -aG docker $USER   then log out and back in'
                if 'permission denied' in status.lower() else
                'sudo systemctl start docker   (the Docker daemon is not running)')
        elif not status:
            say(FAIL, 'The %s container does not exist.' % a.container,
                './scripts/install.sh --robot   then ./start_physios.sh --with-robot')
        elif not status.startswith('Up'):
            say(FAIL, '%s is not running (%s).' % (a.container, status),
                './start_physios.sh --with-robot   (log: var/log/robot-up.log,\n'
                'docker logs %s --tail 50)' % a.container)
        else:
            say(OK, '%s: %s' % (a.container, status))
            src = 'source /opt/ros/$ROS_DISTRO/setup.bash >/dev/null 2>&1; ' \
                  'source ${COLCON_WS:-/root/ros2_ws}/install/setup.bash >/dev/null 2>&1; '
            code, out = run(['docker', 'exec', a.container, 'bash', '-c',
                             src + 'ros2 topic list'], 30)
            topics = [t for t in out.split() if t.startswith('/')]
            say(OK if topics else FAIL, '%d ROS topics visible in the container.' % len(topics),
                None if topics else 'docker logs %s --tail 50' % a.container)
            want = topic or ('/follower_joint_states' if a.role == 'follower' else None)
            if want:
                code, out = run(['docker', 'exec', a.container, 'bash', '-c',
                                 src + 'ros2 topic info -v %s; timeout 4 ros2 topic hz %s'
                                 % (want, want)], 30)
                pubs = [x.split(':', 1)[1].strip() for x in out.splitlines()
                        if x.strip().startswith('Node name:')]
                rate = next((x for x in out.splitlines() if 'average rate' in x), '')
                if rate:
                    say(OK, '%s: %s · publishers %s' % (want, rate.strip(), pubs))
                else:
                    say(FAIL, 'No messages on %s (publishers: %s).' % (want, pubs or 'none'),
                        'Something must publish the %s joint states: bridge.py, or RawBot\n'
                        'Studio\'s own read-only session (Robots → Hardware → Connect,\n'
                        'bridge.py stopped). If bridge.py runs on the host and the container\n'
                        'does not see it: same RMW (rmw_zenoh_cpp) and ROS_DOMAIN_ID on both.'
                        % a.role)
    if port_open('127.0.0.1', 9090):
        say(OK, 'rosbridge answers on :9090.')
    else:
        say(FAIL, 'rosbridge does not answer on :9090.', './start_physios.sh --with-robot')
    if port_open('127.0.0.1', 8080):
        say(OK, 'web_video_server answers on :8080 (camera previews).')
    else:
        say(WARN, 'web_video_server does not answer on :8080: camera previews stay empty '
                  '(recording reads the camera topics directly).')

    print()
    if first_fail:
        text, fix = first_fail[0]
        print('First failing link: %s' % text)
        if fix:
            print('Fix:\n  ' + '\n  '.join(fix.strip().splitlines()))
        return 1
    print('Every link checked passes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
