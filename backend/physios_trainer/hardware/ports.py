# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""USB serial port discovery (Linux sysfs) and port holders (/proc). Read-only.

No device names are assumed: every tty under /sys/class/tty whose device chain reaches a USB
device is listed, whatever it is called (ttyACM*, ttyUSB*, ...). A port is identified by its
USB adapter (vendor, product, serial number), never by its path, because paths change when
cables are re-plugged. Adapters without a serial number fall back to their physical USB
socket and are marked as such.

The roots can be redirected for tests (``RAWBOT_HW_SYSFS``, ``RAWBOT_HW_DEV``,
``RAWBOT_HW_PROC``).
"""

import os
from pathlib import Path


def _roots():
    return (Path(os.environ.get('RAWBOT_HW_SYSFS', '/sys')),
            Path(os.environ.get('RAWBOT_HW_DEV', '/dev')),
            Path(os.environ.get('RAWBOT_HW_PROC', '/proc')))


def _attr(d, name):
    try:
        return (d / name).read_text(encoding='utf-8', errors='replace').strip()
    except OSError:
        return None


def supported():
    sysfs, _, _ = _roots()
    return (sysfs / 'class' / 'tty').is_dir()


def identity_key(port):
    """Stable identity of a USB serial adapter (does not depend on the port path)."""
    vid, pid = port.get('vid') or '?', port.get('pid') or '?'
    if port.get('serial'):
        return 'usb:%s:%s:%s' % (vid, pid, port['serial'])
    return 'usbpath:%s:%s:%s' % (vid, pid, port.get('usb_path') or port['device'])


def list_ports():
    """[{device, name, vid, pid, serial, manufacturer, product, usb_path, by_id, key,
    unique}] for USB serial ports, sorted by device."""
    sysfs, dev, _ = _roots()
    base = sysfs / 'class' / 'tty'
    out = []
    by_id = {}
    sdir = dev / 'serial' / 'by-id'
    if sdir.is_dir():
        for link in sdir.iterdir():
            try:
                by_id[os.path.realpath(link)] = str(link)
            except OSError:
                pass
    try:
        entries = sorted(base.iterdir())
    except OSError:
        return []
    for tty in entries:
        devlink = tty / 'device'
        if not devlink.exists():
            continue  # virtual console / pty
        d = Path(os.path.realpath(devlink))
        usb = None
        for _ in range(6):  # interface -> device (ttyACM) or one more level (ttyUSB)
            if (d / 'idVendor').exists():
                usb = d
                break
            if d.parent == d:
                break
            d = d.parent
        if usb is None:
            continue  # not a USB adapter (e.g. a built-in UART)
        device = str(dev / tty.name)
        p = {
            'device': device,
            'name': tty.name,
            'vid': _attr(usb, 'idVendor'),
            'pid': _attr(usb, 'idProduct'),
            'serial': _attr(usb, 'serial'),
            'manufacturer': _attr(usb, 'manufacturer'),
            'product': _attr(usb, 'product'),
            'usb_path': usb.name,
            'by_id': by_id.get(os.path.realpath(device)),
        }
        p['key'] = identity_key(p)
        p['unique'] = bool(p['serial'])
        p['access'] = _access(device)
        out.append(p)
    return out


def _access(device):
    if not os.path.exists(device):
        return 'missing'
    return 'ok' if os.access(device, os.R_OK | os.W_OK) else 'denied'


def holders(devices, skip_pids=()):
    """{device: [{pid, name, cmdline}]} - processes that have the device open.

    Only processes this user may inspect are seen; that covers the operator's own
    bridge.py. Our own monitor/calibration workers are left out via ``skip_pids``."""
    _, _, proc = _roots()
    want = {}
    for dvc in devices:
        try:
            want[os.path.realpath(dvc)] = dvc
        except OSError:
            pass
    found = {d: [] for d in devices}
    if not want:
        return found
    me = os.getpid()
    skip = set(skip_pids) | {me}
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit()]
    except OSError:
        return found
    for pid in pids:
        if int(pid) in skip:
            continue
        fd_dir = proc / pid / 'fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        hit = None
        for fd in fds:
            try:
                target = os.readlink(fd_dir / fd)
            except OSError:
                continue
            if target in want:
                hit = want[target]
                break
        if hit is None:
            continue
        name = _attr(proc / pid, 'comm') or '?'
        try:
            raw = (proc / pid / 'cmdline').read_bytes().split(b'\0')
            cmd = ' '.join(x.decode('utf-8', 'replace') for x in raw if x)
        except OSError:
            cmd = ''
        script = next((Path(a).name for a in cmd.split()[1:3] if a.endswith('.py')), None)
        found[hit].append({'pid': int(pid), 'name': script or name, 'cmdline': cmd[:200]})
    return found
