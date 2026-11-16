#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""Browser check of the guided robot setup (Robot -> Model file -> Hardware -> Verify ->
Planning) against the dev stack:

    python3 tests/e2e/devstack.py <work> --seed --hardware bridge     # terminal 1
    python3 tests/e2e/setup_flow_check.py http://127.0.0.1:8700 [shots]

A: a built-in robot from Home through hardware and verification to Planning.
B: a custom robot started from its model file: upload first, the name comes from the URDF,
   details afterwards, register, hardware choice, then the verification checklist (which
   holds Planning until the robot is ROS- and hardware-verified and on the robot server).
"""

import re
import sys
import tempfile
from pathlib import Path

from playwright.sync_api import expect, sync_playwright

BASE = sys.argv[1]
SHOTS = Path(sys.argv[2]) if len(sys.argv) > 2 else None
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'backend' / 'trainer_tests'))
from robot_fixtures import arm_zip  # noqa: E402

done = []


def step(t):
    print('>>', t, flush=True)
    done.append(t)


def shot(page, name):
    if SHOTS:
        SHOTS.mkdir(parents=True, exist_ok=True)
        page.wait_for_timeout(700)
        page.screenshot(path=str(SHOTS / f'{name}.png'))


def at(page, n):
    expect(page.locator('[data-setup-flow]')).to_have_attribute('data-setup-at', str(n), timeout=15000)


def nxt(page):
    page.locator('[data-action="setup-next"]').click()


zip_path = Path(tempfile.mkdtemp()) / 'demo_arm_description.zip'
zip_path.write_bytes(arm_zip())

with sync_playwright() as p:
    browser = p.chromium.launch(executable_path='/usr/bin/google-chrome')
    page = browser.new_page(viewport={'width': 1600, 'height': 1000})
    errors = []
    page.on('pageerror', lambda e: errors.append(str(e)))
    page.goto(BASE)
    page.evaluate("localStorage.setItem('rawbot.theme','light')")
    page.reload()

    # ----------------------------------------------------------------- A: built-in robot
    step('A1 Home: choose s101_dataset and confirm (step 1)')
    page.wait_for_selector('[data-robot="s101_dataset"]')
    page.locator('[data-robot="s101_dataset"]').click()
    page.locator('[data-action="confirm-robot"]').click()
    expect(page.locator('[data-status="hb"]')).to_have_text('alive', timeout=10000)
    page.locator('[data-action="home-continue"]').click()

    step('A2 Step 3 hardware: model step skipped, hardware setup shown')
    at(page, 3)
    expect(page.locator('[data-setup-step="2"]')).to_have_attribute('data-step-state', 'skipped')
    expect(page.locator('[data-setup-step="1"]')).to_have_attribute('data-step-state', 'done')
    page.wait_for_selector('[data-hardware-setup]', timeout=15000)
    expect(page.locator('[data-arm-card="leader"]')).to_have_attribute('data-arm-state', 'connected', timeout=15000)
    shot(page, 'setup01_hardware')
    nxt(page)

    step('A3 Step 4 verify: robot server, configuration, arms, calibration')
    at(page, 4)
    expect(page.locator('[data-check="robot_type"]')).to_have_attribute('data-check-state', 'ok')
    expect(page.locator('[data-check="mode"]')).to_have_attribute('data-check-state', 'ok')
    expect(page.locator('[data-check="arms"]')).to_have_attribute('data-check-state', 'ok')
    expect(page.locator('[data-check="leader_calibration"]')).to_be_visible()
    shot(page, 'setup02_verify')

    step('A4 Simulation Only skips the arm checks')
    page.locator('[data-setup-step="3"]').click()
    at(page, 3)
    page.locator('[data-hardware-setup]').get_by_role('tab', name='Simulation Only').click()
    expect(page.locator('[data-hardware-setup]')).to_have_attribute('data-mode', 'simulation', timeout=10000)
    nxt(page)
    at(page, 4)
    expect(page.locator('[data-check="arms"]')).to_have_attribute('data-check-state', 'skip')
    page.locator('[data-setup-step="3"]').click()
    page.locator('[data-hardware-setup]').get_by_role('tab', name='Leader + Follower').click()
    expect(page.locator('[data-hardware-setup]')).to_have_attribute('data-mode', 'leader_follower', timeout=10000)
    nxt(page)
    at(page, 4)

    step('A5 Step 5 summary, then the Planning session')
    nxt(page)
    at(page, 5)
    expect(page.locator('[data-setup-summary]')).to_contain_text('s101_dataset')
    expect(page.locator('[data-setup-summary]')).to_contain_text('Leader + Follower')
    shot(page, 'setup03_ready')
    page.locator('[data-action="setup-to-planning"]').click()
    expect(page.locator('[data-setup-flow]')).to_have_count(0, timeout=10000)
    expect(page.get_by_role('navigation', name='Sessions').get_by_role(
        'button', name=re.compile(r'(^|\s)Planning$'))).to_have_attribute('aria-current', 'page', timeout=10000)
    shot(page, 'setup04_planning')

    # ------------------------------------------------------------------ B: custom robot
    step('B1 Home: Add Custom Robot opens step 2 with the file picker first')
    page.get_by_role('navigation', name='Sessions').get_by_role('button', name=re.compile(r'(^|\s)Home$')).click()
    page.locator('[data-action="add-custom-robot"]').click()
    at(page, 2)
    expect(page.locator('[data-file-picker]')).to_be_visible()
    expect(page.locator('[data-field="name"]')).to_have_count(0)   # no details form before the file
    shot(page, 'setup05_upload')

    step('B2 Upload the model ZIP: name taken from the URDF, checked, 3D view')
    page.locator('[data-file-input]').set_input_files(str(zip_path))
    page.locator('[data-action="setup-upload"]').click()
    page.wait_for_selector('[data-setup-model]', timeout=30000)
    expect(page.locator('[data-setup-validation]')).to_be_visible(timeout=20000)
    expect(page.locator('[data-field="name"]')).to_have_value(re.compile(r'^demo_arm( \d+)?$'))
    name = page.locator('[data-field="name"]').input_value()   # 'demo_arm 2' if one exists
    page.locator('[data-field="manufacturer"]').fill('Lab')
    page.locator('[data-action="setup-save-details"]').click()
    expect(page.get_by_text('Details saved')).to_be_visible(timeout=10000)
    shot(page, 'setup06_model')

    step('B3 Register the model and choose the hardware')
    nxt(page)
    at(page, 3)
    page.locator('[data-setup-mode="follower_only"]').click()
    expect(page.locator('[data-setup-mode="follower_only"]')).to_have_attribute('aria-checked', 'true', timeout=10000)
    shot(page, 'setup07_custom_hardware')
    nxt(page)

    step('B4 Verify: model registered, configuration and stages pending, Planning held')
    at(page, 4)
    expect(page.locator('[data-check="model"]')).to_have_attribute('data-check-state', 'ok', timeout=15000)
    expect(page.locator('[data-check="config"]')).to_have_attribute('data-check-state', 'fail')
    expect(page.locator('[data-check="server"]')).to_have_attribute('data-check-state', 'fail')
    expect(page.locator('[data-action="setup-next"]')).to_be_disabled()
    shot(page, 'setup08_custom_verify')

    step('B5 Robots -> Robot setup tab starts at step 1 with the new custom robot listed')
    page.locator('[data-robots-tab="setup"]').click()
    at(page, 1)
    mine = page.locator('[data-setup-custom]').filter(has_text=re.compile('^%s' % re.escape(name)))
    expect(mine.first).to_be_visible()
    page.locator('[data-theme-option="dark"]').click()
    shot(page, 'setup09_step1_dark')
    mine.first.click()
    nxt(page)
    at(page, 2)
    shot(page, 'setup10_model_dark')
    page.locator('[data-theme-option="light"]').click()
    browser.close()

assert not errors, errors
print('PASSED %d steps' % len(done))
