# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""What RawBot Studio remembers about the operator's hardware (data/workspace/physios/hardware).

registry.json
    devices  - {identity key: {role, robot_type, label, vid, pid, serial, usb_path,
               expected motor ids, last_port, last_validated_at, port_override}}
               The role comes from the operator (validated by a motor scan), never from
               the port name.
    setups   - {robot_type: {mode, topics, env, disconnected}}
profiles/<id>.json
    one calibration profile per file (LeRobot's MotorCalibration fields per motor)
captures/<id>.json
    leader joint captures (Leader Only)
"""

import json
import re
import threading
import time
import uuid
from pathlib import Path

from physios_trainer.util import atomic_write_json, now_iso, read_json

ROLES = ('leader', 'follower')
MODES = {
    'leader_follower': {'label': 'Leader + Follower', 'arms': ('leader', 'follower')},
    'leader_only': {'label': 'Leader Only', 'arms': ('leader',)},
    'follower_only': {'label': 'Follower Only', 'arms': ('follower',)},
    'simulation': {'label': 'Simulation Only', 'arms': ()},
}
DEFAULT_MODE = 'leader_follower'   # the original workflow; existing projects keep it

# SO-101 (and SO-100) arms: six Feetech STS3215 servos, IDs 1-6, on both leader and follower.
# The leader and follower motors are the same model, so motors alone cannot tell them apart:
# the role is bound to the USB adapter and checked against these IDs.
SO101_MOTORS = [['shoulder_pan', 1, 'sts3215'], ['shoulder_lift', 2, 'sts3215'],
                ['elbow_flex', 3, 'sts3215'], ['wrist_flex', 4, 'sts3215'],
                ['wrist_roll', 5, 'sts3215'], ['gripper', 6, 'sts3215']]
STS3215_MODEL_NUMBER = 777
SO_RE = re.compile(r'(so[-_]?10[01]|s10[01])', re.I)


def motor_spec(robot_type):
    """Motors RawBot Studio can scan for this robot type, or None (ROS-only robots)."""
    return [list(m) for m in SO101_MOTORS] if SO_RE.search(robot_type or '') else None


class Registry:

    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / 'profiles').mkdir(exist_ok=True)
        (self.root / 'captures').mkdir(exist_ok=True)
        self.file = self.root / 'registry.json'
        self.lock = threading.RLock()

    # ---- raw --------------------------------------------------------------------
    def _load(self):
        d = read_json(self.file) or {}
        d.setdefault('devices', {})
        d.setdefault('setups', {})
        return d

    def _save(self, d):
        atomic_write_json(self.file, d)

    # ---- setups -------------------------------------------------------------------
    def setup(self, robot_type):
        with self.lock:
            s = dict(self._load()['setups'].get(robot_type or '', {}))
        s.setdefault('mode', DEFAULT_MODE)
        s.setdefault('mode_source', 'default')
        s.setdefault('topics', {})
        s.setdefault('disconnected', [])
        s.setdefault('env', None)
        return s

    def update_setup(self, robot_type, **kw):
        if not robot_type:
            raise ValueError('Select a robot first.')
        if 'mode' in kw and kw['mode'] not in MODES:
            raise ValueError('Unknown hardware configuration "%s".' % kw['mode'])
        with self.lock:
            d = self._load()
            s = d['setups'].setdefault(robot_type, {})
            s.update({k: v for k, v in kw.items() if v is not None or k == 'env'})
            s['updated_at'] = now_iso()
            self._save(d)
        return self.setup(robot_type)

    # ---- devices ------------------------------------------------------------------
    def devices(self):
        with self.lock:
            return self._load()['devices']

    def device_for(self, robot_type, role):
        for key, dvc in self.devices().items():
            if dvc.get('robot_type') == robot_type and dvc.get('role') == role:
                return dict(dvc, key=key)
        return None

    def assign(self, key, role, robot_type, port_info, probe, label=None):
        """Bind an adapter to a role. One adapter per role and robot type: an older
        binding of the same role is replaced (the adapter was swapped)."""
        if role not in ROLES:
            raise ValueError('Role must be leader or follower.')
        with self.lock:
            d = self._load()
            for k in [k for k, v in d['devices'].items()
                      if v.get('robot_type') == robot_type and v.get('role') == role and k != key]:
                d['devices'].pop(k)
            prev = d['devices'].get(key, {})
            d['devices'][key] = {
                'role': role, 'robot_type': robot_type,
                'label': label or prev.get('label') or '%s arm' % role.capitalize(),
                'vid': port_info.get('vid'), 'pid': port_info.get('pid'),
                'serial': port_info.get('serial'), 'usb_path': port_info.get('usb_path'),
                'product': port_info.get('product'), 'unique': port_info.get('unique', True),
                'motor_ids': sorted(int(i) for i in (probe.get('found') or {})),
                'last_port': port_info.get('device'),
                'last_validated_at': now_iso(),
                'port_override': prev.get('port_override'),
                'assigned_at': prev.get('assigned_at') or now_iso(),
            }
            self._save(d)
            return dict(d['devices'][key], key=key)

    def unassign(self, key):
        with self.lock:
            d = self._load()
            if key not in d['devices']:
                raise KeyError('Device not registered.')
            d['devices'].pop(key)
            self._save(d)

    def touch(self, key, **kw):
        with self.lock:
            d = self._load()
            if key in d['devices']:
                d['devices'][key].update(kw)
                self._save(d)

    # ---- calibration profiles ------------------------------------------------------
    def _pid_ok(self, pid):
        return bool(pid) and all(c.isalnum() or c in '-_' for c in pid)

    def save_profile(self, role, robot_type, device_key, motors, name=None, source='calibration'):
        pid = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:4]
        rec = {'id': pid, 'role': role, 'robot_type': robot_type, 'device_key': device_key,
               'name': name or '%s %s' % (role.capitalize(), time.strftime('%d %b %H:%M')),
               'created_at': now_iso(), 'source': source, 'motors': motors}
        atomic_write_json(self.root / 'profiles' / ('%s.json' % pid), rec)
        return rec

    def profiles(self, robot_type=None, role=None):
        out = []
        for p in sorted((self.root / 'profiles').glob('*.json'), reverse=True):
            r = read_json(p)
            if not r:
                continue
            if robot_type and r.get('robot_type') != robot_type:
                continue
            if role and r.get('role') != role:
                continue
            out.append(r)
        return out

    def profile(self, pid):
        if not self._pid_ok(pid):
            raise KeyError(pid)
        r = read_json(self.root / 'profiles' / ('%s.json' % pid))
        if not r:
            raise KeyError('Calibration profile %s not found.' % pid)
        return r

    def rename_profile(self, pid, name):
        r = self.profile(pid)
        r['name'] = (name or '').strip()[:80] or r['name']
        atomic_write_json(self.root / 'profiles' / ('%s.json' % pid), r)
        return r

    def delete_profile(self, pid):
        self.profile(pid)
        (self.root / 'profiles' / ('%s.json' % pid)).unlink()

    # ---- leader captures --------------------------------------------------------------
    def save_capture(self, robot_type, role, names, samples, source):
        if not samples:
            raise ValueError('The capture is empty.')
        if len(samples) > 200000:
            raise ValueError('Capture too long (limit 200 000 samples).')
        cid = time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:4]
        rec = {'id': cid, 'robot_type': robot_type, 'role': role, 'names': list(names),
               'source': source, 'created_at': now_iso(), 'count': len(samples),
               'duration_s': round(float(samples[-1][0]) - float(samples[0][0]), 3)
               if len(samples) > 1 else 0.0,
               'samples': samples}
        atomic_write_json(self.root / 'captures' / ('%s.json' % cid), rec)
        return {k: v for k, v in rec.items() if k != 'samples'}

    def capture(self, cid):
        if not cid or '/' in cid or cid.startswith('.'):
            return None
        try:
            with open(self.root / 'captures' / ('%s.json' % cid)) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def captures(self, robot_type=None):
        out = []
        for p in sorted((self.root / 'captures').glob('*.json'), reverse=True)[:50]:
            try:
                with open(p) as f:
                    r = json.load(f)
            except (OSError, ValueError):
                continue
            if robot_type and r.get('robot_type') != robot_type:
                continue
            r.pop('samples', None)
            out.append(r)
        return out
