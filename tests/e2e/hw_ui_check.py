#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""Browser check of Flexible Robot Setup against the dev stack with simulated arms:

    python3 tests/e2e/devstack.py <work> --seed --hardware bridge     # terminal 1
    python3 tests/e2e/hw_ui_check.py http://127.0.0.1:8700 <work> [shots]

Scenarios: Leader + Follower, leader lost during operation, reconnection, follower lost
during operation, Leader Only, Follower Only (direct control, hand-guided recording, a leader
plugged in later), independent connection and calibration, Simulation Only. The arms are
changed underneath the running stack: the simulated USB adapters (unplug / plug), the
stand-in "bridge.py" holding the ports, and the stand-in teleoperation topic.
"""

import asyncio
import json
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
        page.wait_for_timeout(600)
        page.screenshot(path=str(SHOTS / f'{name}.png'))


def status():
    with urllib.request.urlopen('http://127.0.0.1:8766/hardware/status?robot_type=' + RT,
                                timeout=20) as r:
        return json.load(r)


def rosbridge(service, args):
    import websockets

    async def go():
        async with websockets.connect('ws://127.0.0.1:9090') as ws:
            await ws.send(json.dumps({'op': 'call_service', 'service': service, 'args': args,
                                      'id': 'hwcheck'}))
            while True:
                m = json.loads(await ws.recv())
                if m.get('op') == 'service_response':
                    return m.get('values')
    out = {}

    def run():   # Playwright's sync API owns this thread's event loop
        out['v'] = asyncio.run(go())
    t = threading.Thread(target=run)
    t.start()
    t.join(20)
    return out.get('v')


def teleop(on, follower=True):
    rosbridge('/standin/set_teleop', {'enabled': on, 'follower': follower})


def release(role):
    """Stop the stand-in bridge.py holding this arm's port."""
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


def wait_state(role, want, timeout=15):
    st = None
    for _ in range(int(timeout * 4)):
        st = status()['arms'][role]['state']
        if st in (want if isinstance(want, tuple) else (want,)):
            return
        time.sleep(0.25)
    raise AssertionError('%s state %s, expected %s' % (role, st, want))


def cap(page, key, available):
    expect(page.locator(f'[data-capability="{key}"]')).to_have_attribute(
        'data-available', 'yes' if available else 'no', timeout=12000)


def nav(page, name):
    page.get_by_role('navigation', name='Sessions').get_by_role(
        'button', name=re.compile(r'(^|\s)%s$' % name)).click()


def open_hw(page):
    nav(page, 'Robots')
    page.locator('[data-robots-tab="hardware"]').click()
    page.wait_for_selector('[data-hardware-setup]')


ROLES = {'leader_follower': ('real', 'real'), 'leader_sim': ('real', 'sim'),
         'follower_only': ('sim', 'real'), 'simulation': ('sim', 'sim')}


def mode(page, value):
    """Pick the hardware configuration on Home: one Real/Sim choice per arm. leader_only (older
    setups, no longer offered) is set through the API, then shown as real leader + sim."""
    nav(page, 'Home')
    if value == 'leader_only':
        req = urllib.request.Request(
            'http://127.0.0.1:8766/hardware/setup',
            data=json.dumps({'robot_type': RT, 'mode': 'leader_only'}).encode(),
            headers={'Content-Type': 'application/json'})
        urllib.request.urlopen(req, timeout=10).read()
        expect(page.locator('[data-home-hardware] [data-mode-choice]')).to_have_attribute(
            'data-mode-choice', 'leader_only', timeout=12000)
        return
    box = page.locator('[data-home-hardware]')
    for role, kind in zip(('leader', 'follower'), ROLES[value]):
        b = box.locator(f'[data-arm="{role}"][data-choice="{kind}"]')
        if b.get_attribute('aria-checked') != 'true':
            b.click()
            page.wait_for_timeout(400)
    expect(box.locator('[data-mode-choice]')).to_have_attribute('data-mode-choice', value, timeout=8000)


def record_blocked(page, text):
    nav(page, 'Record')
    expect(page.get_by_text(text).first).to_be_visible(timeout=12000)


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

    # ------------------------------------------------------------------ 1. both arms
    step('1 Leader + Follower: both arms seen through bridge.py, teleop and recording on')
    expect(page.locator('[data-home-hardware]')).to_be_visible(timeout=10000)
    expect(page.locator('[data-home-hardware] [data-mode-choice]')).to_have_attribute('data-mode-choice', 'leader_follower')
    expect(page.locator('[data-home-arm="leader"]')).to_contain_text('Connected', timeout=10000)
    expect(page.locator('[data-home-arm="follower"]')).to_contain_text('Connected')
    shot(page, 'hw01_home_lf')
    page.locator('[data-action="open-hardware"]').click()
    page.wait_for_selector('[data-hardware-setup]')
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', 'connected')
    expect(page.locator('[data-arm-card="follower"]')).to_have_attribute('data-arm-state', 'connected')
    for k in ('teleoperation', 'record', 'collect'):
        cap(page, k, True)
    cap(page, 'hand_guided_record', False)
    shot(page, 'hw02_setup_lf')

    # ------------------------------------------------------- 5. leader lost mid-session
    step('5 Leader disconnected during operation: card lost, recording disabled with reason')
    nav(page, 'Record')
    page.wait_for_timeout(1500)
    teleop(False)
    release('leader')
    world.unplug('ttyACM0')
    wait_state('leader', ('lost', 'not_found'))
    expect(page.get_by_text(re.compile('Leader arm (stopped responding|is not plugged in)')).first).to_be_visible(timeout=12000)
    shot(page, 'hw03_record_leader_lost')
    open_hw(page)
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', re.compile('lost|not_found'))
    expect(page.locator('[data-arm-card="follower"]')).to_have_attribute('data-arm-state', 'connected')
    cap(page, 'record', False)
    cap(page, 'follower_monitor', True)
    shot(page, 'hw04_setup_leader_lost')

    # ------------------------------------------------------------------ 7. reconnect
    step('7 Leader reconnected: plugged back in and bridge.py restarted, recording back on')
    world.plug('ttyACM0', serial='5A46085090', sweep_on_calibrate=True)
    world.hold('ttyACM0')
    teleop(True)
    wait_state('leader', 'connected')
    cap(page, 'record', True)
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', 'connected')

    # ------------------------------------------------------ 6. follower lost mid-session
    step('6 Follower disconnected during operation: inference and recording disabled')
    teleop(True, follower=False)
    release('follower')
    world.unplug('ttyACM1')
    wait_state('follower', ('lost', 'not_found'))
    cap(page, 'record', False)
    cap(page, 'inference', False)
    nav(page, 'Inference')
    expect(page.get_by_text(re.compile('Follower arm')).first).to_be_visible(timeout=12000)
    shot(page, 'hw05_inference_follower_lost')
    world.plug('ttyACM1', serial='5A46081747', sweep_on_calibrate=True)
    world.hold('ttyACM1')
    teleop(True, follower=True)
    wait_state('follower', 'connected')

    # ------------------------------------------------------------------ 2. leader only
    step('2 Leader Only: follower unused, leader monitoring and capture, recording needs the simulated twin')
    mode(page, 'leader_only')
    expect(page.locator('[data-home-arm="follower"]')).to_contain_text('Not used')
    open_hw(page)
    expect(page.locator('[data-arm-card="follower"]')).to_have_attribute('data-arm-state', 'unused')
    cap(page, 'leader_monitor', True)
    cap(page, 'leader_capture', True)
    cap(page, 'record', False)
    cap(page, 'follower_control', False)
    expect(page.locator('[data-leader-capture]')).to_be_visible()
    page.locator('[data-action="capture-start"]').click()
    page.wait_for_timeout(1800)
    page.locator('[data-action="capture-stop"]').click()
    expect(page.get_by_text(re.compile(r'Saved \d+ samples'))).to_be_visible(timeout=10000)
    shot(page, 'hw06_leader_only')
    # only a leader: the simulated follower is the target; bridge.py still publishes the
    # follower state topic here, so the simulated follower cannot take it
    record_blocked(page, 'already publishes /joint_states')

    # ---------------------------------------------------------------- 3. follower only
    step('3 Follower Only: hand-guided recording, direct control (jog) once teleop is off')
    teleop(False)
    release('leader')
    world.unplug('ttyACM0')
    mode(page, 'follower_only')
    open_hw(page)
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', 'unused')
    cap(page, 'hand_guided_record', True)
    cap(page, 'follower_control', True)
    cap(page, 'record', False)
    page.locator('[data-jog-setup] select').select_option(index=1)
    expect(page.locator('[data-jog-limits]')).to_be_visible(timeout=120000)
    page.locator('[data-action="jog-review"]').click()
    for k in ('area_clear', 'estop_ready', 'watching'):
        page.locator(f'[data-jog-check="{k}"]').check()
    page.locator('[data-action="jog-start"]').click()
    page.wait_for_selector('[data-jog-live]', timeout=15000)
    shot(page, 'hw07_follower_jog')
    cap(page, 'policy_rollout', False)
    page.locator('[data-action="jog-stop"]').click()
    expect(page.locator('[data-jog-live]')).to_have_count(0, timeout=15000)
    nav(page, 'Record')
    expect(page.locator('[data-hand-guided]')).to_be_visible(timeout=10000)
    expect(page.get_by_text('Needs Leader + Follower')).to_have_count(0)
    shot(page, 'hw07b_record_hand_guided')

    # --------------------------------------------- 9. leader plugged in later: upgrade
    step('9 Leader connected later: Leader + Follower switches on by itself')
    world.plug('ttyACM0', serial='5A46085090', sweep_on_calibrate=True)
    world.hold('ttyACM0')
    teleop(True)
    nav(page, 'Home')
    expect(page.locator('[data-home-hardware] [data-mode-choice]')).to_have_attribute('data-mode-choice', 'leader_follower', timeout=20000)
    assert status()['mode_source'] == 'auto', status()['mode_source']
    open_hw(page)
    cap(page, 'record', True)
    shot(page, 'hw08_auto_upgrade')

    # ------------------------------------- 8. independent connection and calibration
    step('8 Independent connect and calibration: follower alone, leader untouched')
    teleop(False, follower=False)
    release('follower')
    wait_state('follower', ('detected', 'lost'))
    page.locator('[data-action="connect-follower"]').click()
    expect(page.locator('[data-arm-card="follower"]')).to_have_attribute('data-arm-state', 'connected', timeout=20000)
    assert status()['arms']['follower']['source'] == 'serial'
    assert status()['arms']['leader']['state'] == 'lost' or status()['arms']['leader']['source'] == 'ros'
    shot(page, 'hw09_follower_serial')
    page.locator('[data-action="calibrate-follower"]').click()
    expect(page.locator('[data-cal-phase="ready"]')).to_be_visible(timeout=20000)
    page.locator('[data-action="cal-begin"]').click()
    expect(page.locator('[data-cal-phase="torque_off"]')).to_be_visible(timeout=15000)
    page.locator('[data-action="cal-middle"]').click()
    expect(page.locator('[data-cal-phase="recording"]')).to_be_visible(timeout=15000)
    page.wait_for_timeout(2500)
    shot(page, 'hw10_calibration')
    page.locator('[data-action="cal-finish"]').click()
    expect(page.locator('[data-cal-phase="done"]')).to_be_visible(timeout=20000)
    page.locator('[data-action="cal-close"]').click()
    st = status()
    fp = [x for x in st['arms']['follower']['calibration']['profiles']]
    assert fp, 'no follower profile saved'
    assert not st['arms']['leader']['calibration']['profiles'], 'leader profile appeared'
    page.locator('[data-action="disconnect-follower"]').click()
    expect(page.locator('[data-arm-card="follower"]')).to_have_attribute('data-arm-state', 'disconnected', timeout=10000)
    shot(page, 'hw11_after_calibration')

    # ----------------------------------------------------------- 4. simulation only
    step('4 Simulation Only: arms not used, inference explains why, the simulated robot records')
    mode(page, 'simulation')
    open_hw(page)
    cap(page, 'simulation', True)
    cap(page, 'record', False)
    cap(page, 'inference', False)
    shot(page, 'hw12_simulation')
    # the simulated robot is the recorded robot in Simulation Only (plan_hw_check.py)
    nav(page, 'Record')
    expect(page.locator('[data-plan-setup="compact"]')).to_be_visible(timeout=12000)
    nav(page, 'Inference')
    expect(page.get_by_text('Simulation Only is selected').first).to_be_visible(timeout=10000)
    page.locator('[data-theme-option="dark"]').click()
    page.wait_for_timeout(800)
    shot(page, 'hw13_simulation_dark')
    page.locator('[data-theme-option="light"]').click()

    # ------------------------------------------------------------------ restore
    world.hold('ttyACM1')
    teleop(True, follower=True)
    mode(page, 'leader_follower')
    open_hw(page)
    page.wait_for_timeout(2500)
    page.locator('[data-action="connect-follower"]').click()
    wait_state('follower', 'connected', 20)
    wait_state('leader', 'connected', 20)
    browser.close()

assert not errors, errors
print('PASSED %d scenarios' % len(results))
