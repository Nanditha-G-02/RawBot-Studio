#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""Browser check of the hardware-aware Planning session (Robot & control) against the dev
stack with simulated arms:

    python3 tests/e2e/devstack.py <work> --seed --hardware bridge     # terminal 1
    python3 tests/e2e/plan_hw_check.py http://127.0.0.1:8700 <work> [shots]

P1 Leader + Follower, P2 Physical + Simulated (mirror, comparison, leader drives the
simulation), P3 Leader Only (records through the simulated robot), P4 Follower Only (hand
guiding, keyboard direct control of the follower), P5 Simulation Only (GUI, keyboard,
script, replay, trajectory capture, a plan episode recorded with its setup), both themes.
"""

import asyncio
import json
import math
import os
import re
import signal
import sys
import threading
import time
import urllib.request
from pathlib import Path

from playwright.sync_api import expect, sync_playwright

BASE = sys.argv[1]
WORK = Path(sys.argv[2]).resolve()
SHOTS = Path(sys.argv[3]) if len(sys.argv) > 3 else None
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'backend' / 'trainer_tests'))
from hw_fixtures import HwWorld  # noqa: E402

RT = 's101_dataset'
world = HwWorld(WORK / 'usb')
results = []


def step(name):
    print('>>', name, flush=True)
    results.append(name)


def shot(page, name):
    if SHOTS:
        SHOTS.mkdir(parents=True, exist_ok=True)
        page.wait_for_timeout(700)
        page.screenshot(path=str(SHOTS / f'{name}.png'))


def status():
    with urllib.request.urlopen('http://127.0.0.1:8766/hardware/status?robot_type=' + RT,
                                timeout=20) as r:
        return json.load(r)


def rosbridge(service, args):
    import websockets

    async def go():
        async with websockets.connect('ws://127.0.0.1:9090', max_size=None) as ws:
            await ws.send(json.dumps({'op': 'call_service', 'service': service, 'args': args,
                                      'id': 'plancheck'}))
            while True:
                m = json.loads(await ws.recv())
                if m.get('op') == 'service_response':
                    return m.get('values')
    out = {}

    def run():   # Playwright's sync API owns this thread's event loop
        out['v'] = asyncio.run(go())
    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(30)
    return out.get('v')


def planning(cmd, payload):
    v = rosbridge('/planning/command', {'command': cmd, 'payload_json': json.dumps(payload)})
    assert v and v.get('success'), v
    return json.loads(v['result_json'])


def teleop(on, follower=True):
    rosbridge('/standin/set_teleop', {'enabled': on, 'follower': follower})


def release(role):
    for h in status()['arms'][role]['holders']:
        try:
            os.kill(h['pid'], signal.SIGTERM)
        except OSError:
            pass
    for _ in range(40):
        if not status()['arms'][role]['holders']:
            return
        time.sleep(0.25)
    raise AssertionError('%s port still held' % role)


def wait_for(fn, timeout=15, what=''):
    v = None
    for _ in range(int(timeout * 4)):
        v = fn()
        if v:
            return v
        time.sleep(0.25)
    raise AssertionError('timeout: %s' % what)


def nav(page, name):
    page.get_by_role('navigation', name='Sessions').get_by_role(
        'button', name=re.compile(r'(^|\s)%s$' % name)).click()


def mode(page, value):
    nav(page, 'Home')
    page.locator(f'[data-home-mode="{value}"]').click()
    expect(page.locator(f'[data-home-mode="{value}"]')).to_have_attribute('aria-checked', 'true',
                                                                          timeout=8000)


def panel(page):
    nav(page, 'Planning')
    page.wait_for_selector('[data-plan-setup="full"]', timeout=12000)
    if page.locator('[data-plan-choice="action"]').count() == 0:
        page.locator('[data-action="plan-setup-toggle"]').click()
    return page.locator('[data-plan-setup="full"]')


def selection(page, want):
    expect(page.locator('[data-plan-setup="full"]')).to_have_attribute(
        'data-plan-selection', want, timeout=12000)


def pick(page, kind, option):
    page.locator(f'[data-plan-choice="{kind}"] [data-option="{option}"]').click()
    expect(page.locator(f'[data-plan-choice="{kind}"] [data-option="{option}"]')).to_have_attribute(
        'aria-checked', 'true', timeout=8000)


def option(page, kind, opt, available):
    expect(page.locator(f'[data-plan-choice="{kind}"] [data-option="{opt}"]')).to_have_attribute(
        'data-available', 'yes' if available else 'no', timeout=12000)


def record(page, ready):
    expect(page.locator('[data-plan-record]')).to_have_attribute(
        'data-plan-record', 'ready' if ready else 'blocked', timeout=12000)


def twin(page, on):
    t = page.locator('[data-plan-sim-twin]')
    if t.get_attribute('data-plan-sim-twin') != ('on' if on else 'off'):
        t.get_by_role('switch').click()
    expect(t).to_have_attribute('data-plan-sim-twin', 'on' if on else 'off', timeout=8000)


def sim_pos(joint):
    st = status()['sim']
    return st['positions'][st['names'].index(joint)]


with sync_playwright() as p:
    browser = p.chromium.launch(executable_path='/usr/bin/google-chrome')
    page = browser.new_page(viewport={'width': 1600, 'height': 960})
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto(BASE)
    page.evaluate("localStorage.setItem('rawbot.theme','light')")
    page.reload()
    page.wait_for_selector(f'[data-robot="{RT}"]')
    page.locator(f'[data-robot="{RT}"]').click()
    page.locator('[data-action="confirm-robot"]').click()
    expect(page.locator('[data-status="hb"]')).to_have_text('alive', timeout=10000)

    # ------------------------------------------------------------- P1 leader + follower
    step('P1 Leader + Follower: teleoperation to the follower, scripts off, ready to record')
    panel(page)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'leader_follower', timeout=12000)
    selection(page, 'leader|follower|follower')
    option(page, 'action', 'leader', True)
    option(page, 'action', 'script', False)
    expect(page.locator('[data-plan-choice="action"] [data-option="script"]')).to_contain_text('simulation off')
    option(page, 'action', 'hand_guided', False)
    option(page, 'target', 'sim', False)
    expect(page.locator('[data-plan-camera]')).to_have_count(2, timeout=8000)
    record(page, True)
    expect(page.locator('[data-plan-robot="leader"]')).to_have_attribute('data-robot-state', 'connected')
    expect(page.locator('[data-plan-robot="follower"]')).to_have_attribute('data-robot-state', 'connected')
    shot(page, 'pl01_leader_follower')

    # ------------------------------------------------------ P2 physical + simulated
    step('P2 Physical + Simulated: mirror and real-vs-sim, then the leader drives the simulation')
    twin(page, True)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'physical_sim', timeout=12000)
    expect(page.locator('[data-plan-robot="sim"]')).to_have_attribute('data-robot-state', 'running')
    wait_for(lambda: status()['sim']['drive'] == 'follower', what='mirror drive')
    expect(page.locator('[data-plan-compare]')).to_be_visible(timeout=12000)
    option(page, 'observation', 'sim', False)          # the physical follower owns the state
    shot(page, 'pl02_physical_sim_mirror')
    pick(page, 'target', 'sim')
    selection(page, 'leader|sim|follower')
    wait_for(lambda: status()['sim']['drive'] == 'leader', what='leader drive')
    record(page, True)
    shot(page, 'pl03_leader_drives_sim')

    # -------------------------------------------------------------------- P3 leader only
    step('P3 Leader Only: only a leader, so its simulated follower is the target by itself')
    teleop(True, follower=False)
    release('follower')
    world.unplug('ttyACM1')
    mode(page, 'leader_only')
    panel(page)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'leader_sim', timeout=12000)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-auto', 'yes')
    selection(page, 'leader|sim|sim')
    wait_for(lambda: status()['sim']['state_topic'] == '/joint_states', what='sim owns state')
    record(page, True)
    expect(page.locator('[data-plan-sim-twin]')).to_have_count(0)
    expect(page.locator('[data-plan-check="mapping"]')).to_have_attribute('data-check-state', 'ok', timeout=10000)
    expect(page.locator('[data-plan-check="mapping"]')).to_contain_text('6/6')
    expect(page.locator('[data-plan-check="gripper"]')).to_have_attribute('data-check-state', 'ok')
    expect(page.locator('[data-plan-robot-model]')).to_contain_text('6 joints · gripper')
    option(page, 'target', 'follower', False)
    expect(page.locator('[data-plan-choice="target"] [data-option="follower"]')).to_be_hidden()
    option(page, 'action', 'replay', True)
    shot(page, 'pl04_leader_only')
    nav(page, 'Record')
    expect(page.locator('[data-record-setup="leader|sim"]')).to_be_visible(timeout=10000)
    expect(page.locator('[data-plan-setup="compact"]')).to_be_visible()
    expect(page.locator('[data-control="Start"]')).to_be_enabled(timeout=10000)
    shot(page, 'pl05_record_leader_only')

    # ------------------------------------------------------------------ P4 follower only
    step('P4 Follower Only: hand guiding, keyboard direct control of the follower')
    teleop(False, follower=True)
    release('leader')
    world.unplug('ttyACM0')
    world.plug('ttyACM1', serial='5A46081747', sweep_on_calibrate=True)
    world.hold('ttyACM1')
    mode(page, 'follower_only')
    panel(page)
    twin(page, False)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'follower_only', timeout=12000)
    selection(page, 'hand_guided|follower|follower')
    record(page, True)
    option(page, 'action', 'leader', False)
    for k in ('gui', 'keyboard', 'gamepad'):
        option(page, 'action', k, True)
    pick(page, 'action', 'keyboard')
    selection(page, 'keyboard|follower|follower')
    record(page, False)
    expect(page.locator('[data-plan-record-reason]')).to_contain_text('following')
    expect(page.locator('[data-jog-setup], [data-jog-unavailable]').first).to_be_visible(timeout=10000)
    shot(page, 'pl06_follower_only_keyboard')
    pick(page, 'action', 'hand_guided')
    record(page, True)
    nav(page, 'Record')
    expect(page.locator('[data-hand-guided]')).to_be_visible(timeout=10000)

    # --------------------------------------------------------------- P5 simulation only
    step('P5 Simulation Only: GUI, keyboard, script, replay and capture of the simulated robot')
    mode(page, 'simulation')
    panel(page)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'simulation', timeout=12000)
    selection(page, 'gui|sim|sim')
    # bridge.py still publishes the follower state topic: the simulated robot does not take it
    record(page, False)
    expect(page.locator('[data-plan-record-reason]')).to_contain_text('already publishes /joint_states')
    teleop(False, follower=False)
    release('follower')
    record(page, True)
    option(page, 'action', 'leader', False)
    option(page, 'action', 'hand_guided', False)
    option(page, 'target', 'follower', False)
    expect(page.locator('[data-plan-robot="leader"]')).to_have_attribute('data-robot-state', re.compile('unused|.*'))
    expect(page.locator('[data-sim-sliders]')).to_be_visible(timeout=10000)
    page.locator('[data-action="sim-home"]').click()
    s = page.locator('[data-sim-slider="elbow_flex"]')
    s.focus()
    for _ in range(40):
        page.keyboard.press('ArrowRight')
    wait_for(lambda: sim_pos('elbow_flex') > 0.3, what='slider moves the simulated elbow')
    shot(page, 'pl07_sim_gui')

    pick(page, 'action', 'keyboard')
    page.locator('[data-sim-held="keyboard"]').wait_for()
    page.mouse.click(5, 900)
    before = sim_pos('shoulder_pan')
    page.keyboard.down('q')
    page.wait_for_timeout(900)
    expect(page.locator('[data-held-joint="shoulder_pan"]')).to_have_class(re.compile('bg-sky-50'))
    page.keyboard.up('q')
    after = sim_pos('shoulder_pan')
    assert after > before + 0.3, (before, after)
    page.wait_for_timeout(700)
    stopped = sim_pos('shoulder_pan')
    page.wait_for_timeout(700)
    assert abs(sim_pos('shoulder_pan') - stopped) < 1e-3, 'kept moving after release'
    shot(page, 'pl08_sim_keyboard')

    pick(page, 'action', 'script')
    page.locator('[data-action="sim-add-waypoint"]').click()
    page.locator('[data-action="sim-home"]').click()
    page.wait_for_timeout(1200)
    page.locator('[data-action="sim-add-waypoint"]').click()
    expect(page.locator('[data-sim-waypoints="2"]')).to_be_visible()
    page.locator('[data-action="sim-play-script"]').click()
    wait_for(lambda: status()['sim'].get('script'), what='script runs')
    wait_for(lambda: status()['sim']['script']['done'], timeout=20, what='script done')
    assert abs(sim_pos('shoulder_pan')) < 1e-3
    shot(page, 'pl09_sim_script')

    page.locator('[data-action="sim-capture-start"]').click()
    pick(page, 'action', 'keyboard')
    page.mouse.click(5, 900)
    page.keyboard.down('w')
    page.wait_for_timeout(700)
    page.keyboard.up('w')
    page.locator('[data-action="sim-capture-stop"]').click()
    expect(page.get_by_text(re.compile(r'Trajectory saved \(\d+ samples'))).to_be_visible(timeout=10000)

    pick(page, 'action', 'replay')
    page.locator('[data-sim-replay] select').first.select_option(index=1)
    page.locator('[data-action="sim-replay"]').click()
    expect(page.locator('[data-sim-replay-state]')).to_be_visible(timeout=20000)
    shot(page, 'pl10_sim_replay_dataset')
    page.locator('[data-replay-from="dataset"] button', has_text='Saved capture').click()
    page.locator('[data-sim-replay] select').first.select_option(index=1)
    page.locator('[data-action="sim-replay"]').click()
    wait_for(lambda: 'capture' in (status()['sim'].get('replay') or {}).get('label', ''), what='capture replay')

    step('P5b Simulation Only: a plan episode recorded with the simulated robot keeps its setup')
    pick(page, 'action', 'gui')
    nav(page, 'Record')
    page.get_by_role('tab', name='Plan mode').click()
    expect(page.locator('[data-plan-setup="compact"]')).to_be_visible(timeout=10000)
    expect(page.locator('[data-sim-sliders]')).to_be_visible()
    expect(page.locator('[data-control="Start"]')).to_be_enabled(timeout=10000)
    shot(page, 'pl11_record_simulation')
    page.locator('[data-control="Start"]').click()
    expect(page.get_by_text(re.compile(r'E\d+ saved and validated'))).to_be_visible(timeout=30000)
    plans = planning('plan.list', {})
    pid = plans['active_plan_id']
    st = planning('record.state', {'plan_id': pid})
    eps = st['progress']['episodes']
    got = [a.get('setup') for e in eps.values() for a in e.get('attempts', []) if a.get('setup')]
    assert got and got[-1]['config'] == 'simulation' and got[-1]['target'] == 'sim', got
    assert got[-1]['action_param'] == 'follower', got

    page.locator('[data-theme-option="dark"]').click()
    page.wait_for_timeout(800)
    shot(page, 'pl12_record_simulation_dark')
    panel(page)
    shot(page, 'pl13_planning_simulation_dark')
    page.locator('[data-theme-option="light"]').click()

    # ------------------------------------- P6 physical leader + simulated follower
    step('P6 Physical Leader + Simulated Follower: one arm over serial drives the simulated follower and is recorded')
    world.plug('ttyACM0', serial='5A46085090', sweep_on_calibrate=True)   # free port, no bridge.py
    mode(page, 'leader_sim')
    nav(page, 'Robots')
    page.locator('[data-robots-tab="hardware"]').click()
    page.wait_for_selector('[data-hardware-setup]')
    expect(page.locator('[data-arm-card="follower"]')).to_have_attribute('data-arm-state', 'unused', timeout=10000)
    page.locator('[data-action="connect-leader"]').click()
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', 'connected', timeout=20000)
    assert status()['arms']['leader']['source'] == 'serial'
    panel(page)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'leader_sim', timeout=12000)
    selection(page, 'leader|sim|sim')
    expect(page.locator('[data-plan-robot="follower"]')).to_have_count(0)        # no physical follower
    expect(page.locator('[data-plan-robot="sim"]')).to_contain_text('Simulated follower')
    expect(page.locator('[data-plan-sim-twin]')).to_have_count(0)
    option(page, 'target', 'follower', False)
    expect(page.locator('[data-leader-sync]')).to_have_attribute('data-leader-sync', 'live', timeout=12000)
    world.set('ttyACM0', actual={'1': 3072, '2': 2048, '3': 2048, '4': 2048, '5': 2048, '6': 2560})
    wait_for(lambda: abs(sim_pos('shoulder_pan') - math.pi / 2) < 0.01, what='simulated follower follows the leader')
    wait_for(lambda: abs(sim_pos('gripper') - math.pi / 4) < 0.01, what='gripper follows')
    expect(page.locator('[data-sync-joint="shoulder_pan"]')).to_contain_text('1.571', timeout=10000)
    record(page, True)
    assert status()['planning']['record']['action_param'] == 'leader_state'
    shot(page, 'pl14_leader_sim')
    nav(page, 'Record')
    expect(page.locator('[data-record-setup="leader|sim"]')).to_be_visible(timeout=10000)
    expect(page.locator('[data-leader-sync]')).to_be_visible()
    expect(page.locator('[data-control="Start"]')).to_be_enabled(timeout=10000)
    shot(page, 'pl15_record_leader_sim')
    page.locator('[data-control="Start"]').click()
    expect(page.get_by_text(re.compile(r'E\d+ saved and validated'))).to_be_visible(timeout=30000)
    st = planning('record.state', {'plan_id': pid})
    got = [a.get('setup') for e in st['progress']['episodes'].values()
           for a in e.get('attempts', []) if a.get('setup')]
    assert got[-1]['config'] == 'leader_sim' and got[-1]['action_param'] == 'leader_state', got
    page.locator('[data-theme-option="dark"]').click()
    panel(page)
    shot(page, 'pl16_leader_sim_dark')
    page.locator('[data-theme-option="light"]').click()
    nav(page, 'Robots')
    page.locator('[data-robots-tab="hardware"]').click()
    page.locator('[data-action="disconnect-leader"]').click()
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', 'disconnected', timeout=10000)

    # ------------------------------- P7 simulated leader -> physical follower (A3)
    step('P7 Simulated leader drives the physical follower: follow, record, pause, end')
    teleop(False, follower=True)                 # bridge.py with the follower, no teleop
    world.hold('ttyACM1')
    mode(page, 'follower_only')
    wait_for(lambda: status()['arms']['follower']['state'] == 'connected', 20, 'follower on ROS')
    panel(page)
    pick(page, 'action', 'gui')
    selection(page, 'gui|follower|follower')
    record(page, False)
    expect(page.locator('[data-plan-record-reason]')).to_contain_text('following')
    expect(page.locator('[data-sim-sliders]')).to_be_visible(timeout=10000)
    page.locator('[data-jog-setup="follow"] select').select_option(index=1)
    expect(page.locator('[data-jog-limits]')).to_be_visible(timeout=120000)
    page.locator('[data-action="jog-review"]').click()
    for k in ('area_clear', 'estop_ready', 'watching'):
        page.locator(f'[data-jog-check="{k}"]').check()
    page.locator('[data-action="jog-start"]').click()
    page.wait_for_selector('[data-follow]', timeout=15000)
    jog = wait_for(lambda: (status()['sessions'].get('jog') or {}).get('phase') == 'ready'
                   and status()['sessions']['jog'], 15, 'follow session ready')
    page.locator('[data-action="follow-toggle"]').click()
    expect(page.locator('[data-follow]')).to_have_attribute('data-follow', 'on', timeout=10000)
    # switching on lined the simulated leader up with the arm: no jump
    m0 = status()['sessions']['jog']['measured']['shoulder_pan']
    assert abs(sim_pos('shoulder_pan') - m0) < 0.1, 'simulated leader not lined up'
    goal = m0 + (0.12 if m0 < 0 else -0.12)
    urllib.request.urlopen(urllib.request.Request(
        'http://127.0.0.1:8766/hardware/sim?robot_type=' + RT,
        data=json.dumps({'command': 'targets', 'targets': {'shoulder_pan': goal}}).encode(),
        headers={'Content-Type': 'application/json'}), timeout=10)
    wait_for(lambda: abs(status()['sessions']['jog']['measured']['shoulder_pan'] - goal) < 0.03,
             20, 'physical follower follows the simulated leader')
    record(page, True)
    expect(page.locator('[data-plan-config]')).to_have_attribute('data-plan-config', 'sim_leader')
    expect(page.locator('[data-plan-sim-twin]')).to_have_count(0)
    shot(page, 'pl17_virtual_teleop')
    nav(page, 'Record')
    expect(page.locator('[data-record-setup="gui|follower"]')).to_be_visible(timeout=10000)
    page.get_by_role('tab', name='Plan mode').click()
    expect(page.locator('[data-control="Start"]')).to_be_enabled(timeout=10000)
    page.locator('[data-control="Start"]').click()
    expect(page.get_by_text(re.compile(r'E\d+ saved and validated'))).to_be_visible(timeout=30000)
    st = planning('record.state', {'plan_id': pid})
    got = [a.get('setup') for e in st['progress']['episodes'].values()
           for a in e.get('attempts', []) if a.get('setup')]
    assert got[-1]['action'] == 'gui' and got[-1]['target'] == 'follower', got
    assert got[-1]['action_param'] == 'leader' and got[-1]['robot']['names'], got
    meta = Path(planning('meta', {})['dataset_root']) / st['progress']['dataset_id'] / 'meta'
    lines = (meta / 'rawbot_recordings.jsonl').read_text().splitlines()
    assert json.loads(lines[-1])['setup']['action'] == 'gui'
    panel(page)
    page.locator('[data-action="follow-toggle"]').click()
    expect(page.locator('[data-follow]')).to_have_attribute('data-follow', 'off', timeout=10000)
    page.locator('[data-action="jog-stop"]').click()
    expect(page.locator('[data-follow]')).to_have_count(0, timeout=15000)
    pick(page, 'action', 'hand_guided')

    # --------------------------- P8 simulated cameras + tool-space control (sim only)
    step('P8 Simulation Only without cameras: simulated cameras, tool pad, workspace hold')
    teleop(False, follower=False)
    release('follower')
    rosbridge('/standin/set_teleop', {'enabled': False, 'follower': False, 'cameras': False})
    mode(page, 'simulation')
    panel(page)
    pick(page, 'action', 'gui')
    selection(page, 'gui|sim|sim')
    wait_for(lambda: len(status()['planning'].get('sim_cameras') or []) == 2, 20, 'sim cameras listed')
    views = {c['name']: c['view'] for c in status()['planning']['sim_cameras']}
    assert views == {'global': 'scene', 'gripper': 'wrist'}, views
    # this page renders and publishes them; they are live only once frames arrive
    for cam in ('global', 'gripper'):
        expect(page.locator(f'[data-plan-camera="{cam}"]')).to_have_attribute('data-camera-sim', 'yes', timeout=15000)
        expect(page.locator(f'[data-plan-camera="{cam}"]')).to_have_attribute('data-camera-live', 'yes', timeout=20000)
    record(page, True)
    expect(page.locator('[data-tool-pad]')).to_be_visible(timeout=10000)
    page.locator('[data-action="sim-home"]').click()
    page.wait_for_timeout(1500)
    z0 = status()['sim']['tool_mm'][2]
    page.locator('[data-tool="z+"]').click()
    page.locator('[data-tool="z+"]').click()
    wait_for(lambda: status()['sim']['tool_mm'][2] > z0 + 12, 10, 'tool pad lifts the tool')
    shot(page, 'pl18_sim_cameras_tool')
    # a pose through the table is held, with the reason shown
    urllib.request.urlopen(urllib.request.Request(
        'http://127.0.0.1:8766/hardware/sim?robot_type=' + RT,
        data=json.dumps({'command': 'targets', 'targets': {'shoulder_lift': 1.2, 'elbow_flex': 1.2}}).encode(),
        headers={'Content-Type': 'application/json'}), timeout=10)
    expect(page.locator('[data-sim-blocked]')).to_be_visible(timeout=10000)
    assert status()['sim']['tool_mm'][2] >= -1, status()['sim']['tool_mm']
    page.locator('[data-theme-option="dark"]').click()
    shot(page, 'pl19_sim_cameras_dark')
    page.locator('[data-theme-option="light"]').click()
    page.locator('[data-action="sim-home"]').click()
    rosbridge('/standin/set_teleop', {'enabled': False, 'follower': False, 'cameras': True})
    for cam in ('global', 'gripper'):
        expect(page.locator(f'[data-plan-camera="{cam}"]')).not_to_have_attribute('data-camera-sim', 'yes', timeout=20000)

    # ------------------------------------------------------------------ restore
    world.hold('ttyACM0')
    world.hold('ttyACM1')
    teleop(True, follower=True)
    mode(page, 'leader_follower')
    nav(page, 'Robots')
    page.locator('[data-robots-tab="hardware"]').click()
    page.wait_for_timeout(2500)
    page.locator('[data-action="connect-leader"]').click()
    wait_for(lambda: status()['arms']['leader']['state'] == 'connected', 20, 'leader back')
    panel(page)
    twin(page, False)
    selection(page, 'leader|follower|follower')
    browser.close()

assert not errors, errors
print('PASSED %d scenarios' % len(results))
