# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Test-only: a fake USB serial world (sysfs + /dev) and simulated SO-101 motor buses.

    w = HwWorld(tmp_path)
    w.plug('ttyACM0', serial='5A46085090')      # adapter appears, six motors answer
    w.unplug('ttyACM0')
    p = w.hold('ttyACM0')                       # a process holding the port (like bridge.py)

The service is pointed at it with RAWBOT_HW_SYSFS / RAWBOT_HW_DEV / RAWBOT_HW_SIM and
RAWBOT_HW_PYTHON (the worker runs with this Python; the simulated bus needs no LeRobot).
"""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


class FakeRos:
    """Stand-in for manager.RosWatch: topic -> (t, names, values); publishers per topic."""

    def __init__(self):
        self.msgs = {}
        self.pubs = {}
        self.connected = True
        self.error = None
        self.url = 'ws://127.0.0.1:9090'
        self.topics = {}
        self.unknown = set()    # topics whose publishers rosapi cannot tell

    def watch(self, topics, publishers_of=()):
        self.topics = dict(topics)
        self.pub_topics = set(t for t in publishers_of if t)

    def publish(self, topic, names, values, publisher='/so101_bridge'):
        self.msgs[topic] = (time.time(), list(names), list(values))
        if publisher:
            self.pubs[topic] = [publisher]

    def silence(self, topic):
        self.msgs.pop(topic, None)
        self.pubs.pop(topic, None)

    def sample(self, topic):
        hit = self.msgs.get(topic)
        if not hit:
            return None, [], []
        return time.time() - hit[0], hit[1], hit[2]

    def foreign_publishers(self, topic):
        if topic in self.unknown:
            return None
        return list(self.pubs.get(topic, []))


class HwWorld:

    def __init__(self, root):
        self.root = Path(root)
        self.sys = self.root / 'sys'
        self.dev = self.root / 'dev'
        (self.sys / 'class' / 'tty').mkdir(parents=True, exist_ok=True)
        (self.dev / 'serial' / 'by-id').mkdir(parents=True, exist_ok=True)
        self.sim = self.root / 'hwsim.json'
        self.sim.write_text(json.dumps({'ports': {}}))
        self.procs = []
        self.usb_n = 0

    def env(self):
        return {'RAWBOT_HW_SYSFS': str(self.sys), 'RAWBOT_HW_DEV': str(self.dev),
                'RAWBOT_HW_SIM': str(self.sim), 'RAWBOT_HW_PYTHON': sys.executable}

    def apply_env(self, monkeypatch):
        for k, v in self.env().items():
            monkeypatch.setenv(k, v)

    def _data(self):
        return json.loads(self.sim.read_text())

    def _store(self, d):
        tmp = self.sim.with_suffix('.tmp')
        tmp.write_text(json.dumps(d))
        os.replace(tmp, self.sim)

    def device(self, name):
        return str(self.dev / name)

    def plug(self, name, serial='SN-%d', vid='1a86', pid='55d3', motors=None, product='USB Single Serial',
             **sim):
        self.usb_n += 1
        usb = '1-%d' % self.usb_n
        if '%d' in (serial or ''):
            serial = serial % self.usb_n
        udev = self.sys / 'devices' / 'pci0000:00' / 'usb1' / usb
        intf = udev / ('%s:1.0' % usb)
        (intf / 'tty' / name).mkdir(parents=True, exist_ok=True)
        for k, v in (('idVendor', vid), ('idProduct', pid), ('manufacturer', 'QinHeng'),
                     ('product', product)):
            (udev / k).write_text(v + '\n')
        if serial:
            (udev / 'serial').write_text(serial + '\n')
        tty = self.sys / 'class' / 'tty' / name
        tty.mkdir(parents=True, exist_ok=True)
        link = tty / 'device'
        if link.exists() or link.is_symlink():
            link.unlink()
        link.symlink_to(intf)
        Path(self.device(name)).write_text('')
        if serial:
            byid = self.dev / 'serial' / 'by-id' / ('usb-QinHeng_%s-if00' % serial)
            if byid.is_symlink():
                byid.unlink()
            byid.symlink_to(self.device(name))
        d = self._data()
        prev = d['ports'].get(self.device(name), {})
        state = {'motors': {str(i): 777 for i in (motors or range(1, 7))},
                 'actual': {str(i): 1800 + 60 * i for i in range(1, 7)},
                 'calibration': prev.get('calibration', {}), 'torque': {}, 'unplugged': False}
        state.update(sim)
        d['ports'][self.device(name)] = state
        self._store(d)
        return serial

    def unplug(self, name):
        tty = self.sys / 'class' / 'tty' / name
        shutil.rmtree(tty, ignore_errors=True)
        try:
            os.unlink(self.device(name))
        except OSError:
            pass
        for link in (self.dev / 'serial' / 'by-id').iterdir():
            if not link.exists():
                link.unlink()
        d = self._data()
        if self.device(name) in d['ports']:
            d['ports'][self.device(name)]['unplugged'] = True
        self._store(d)

    def set(self, name, **kw):
        d = self._data()
        d['ports'][self.device(name)].update(kw)
        self._store(d)

    def state(self, name):
        return self._data()['ports'][self.device(name)]

    def hold(self, name, script='bridge.py'):
        """A process that keeps the port open, named like the operator's bridge.py."""
        code = ("import sys,time\nf=open(sys.argv[1],'rb')\nprint('ok',flush=True)\n"
                "time.sleep(600)\n")
        sp = self.root / script
        sp.write_text(code)
        p = subprocess.Popen([sys.executable, str(sp), self.device(name)],
                             stdout=subprocess.PIPE, text=True)
        p.stdout.readline()
        self.procs.append(p)
        return p

    def close(self):
        for p in self.procs:
            p.kill()
            p.wait()
