# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Talks to an SO-101 arm's Feetech motor bus. Started by the training service with the
operator's own Python environment (the LeRobot install that already drives the arm):

    python -I hw_worker.py probe     <config.json>   read-only scan, prints one JSON line
    python -I hw_worker.py monitor   <config.json>   read-only joint stream (status file)
    python -I hw_worker.py calibrate <config.json>   guided calibration / apply a profile
    python -I hw_worker.py ports                     pyserial port list (non-Linux hosts)

It uses LeRobot's FeetechMotorsBus. It only opens a port that no other program holds (the
service checks this first; bridge.py keeps ownership of the ports while it runs).

What each command may change on the motors:
  probe, monitor  nothing - reads only (ping, model, firmware, position, torque, calibration)
  calibrate       torque OFF (no motion is ever commanded; the arm must be supported), then the
                  homing offset and position limits in the motors' EEPROM, exactly like
                  `lerobot-calibrate`. Cancel or a lost heartbeat writes the original values back.

``RAWBOT_HW_SIM=<file.json>`` replaces the motor bus with a simulated one (tests and the
development stack). Nothing else differs.
"""

import json
import math
import os
import sys
import time

RESOLUTION = 4096       # STS3215 encoder
FULL_TURN = 'wrist_roll'  # LeRobot records no range for it (0..4095)
OUT = 'RAWBOT_HW '


def now():
    return time.time()


def write_status(path, st):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(st, f)
    os.replace(tmp, path)


def read_control(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


# ------------------------------------------------------------------- buses ---
class LeRobotBus:
    """Thin wrapper over lerobot.motors.feetech.FeetechMotorsBus (LeRobot 0.4 - 0.6)."""

    def __init__(self, port, motors, baudrate=None):
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        self.names = [m[0] for m in motors]
        self.ids = {m[0]: int(m[1]) for m in motors}
        self.bus = FeetechMotorsBus(port=port, motors={
            m[0]: Motor(int(m[1]), m[2], MotorNormMode.RANGE_M100_100) for m in motors})
        self.bus.connect(handshake=False)   # opens the port; writes nothing
        if baudrate:
            self.bus.set_baudrate(int(baudrate))
        self.baudrate = self.bus.get_baudrate()

    def ping(self):
        found = self.bus.broadcast_ping(num_retry=2) or {}
        if not found:   # some adapters drop broadcast replies: ask each expected ID
            for n, i in self.ids.items():
                try:
                    mn = self.bus.ping(i, num_retry=1)
                except Exception:  # noqa: BLE001
                    mn = None
                if mn is not None:
                    found[i] = mn
        return {int(k): int(v) for k, v in found.items()}

    def firmware(self, ids):
        try:
            return {int(k): str(v) for k, v in
                    (self.bus._read_firmware_version(list(ids)) or {}).items()}
        except Exception:  # noqa: BLE001
            return {}

    def read(self, data, name):
        return int(self.bus.read(data, name, normalize=False, num_retry=1))

    def sync_pos(self, names):
        return {k: int(v) for k, v in self.bus.sync_read('Present_Position', list(names),
                                                         normalize=False, num_retry=2).items()}

    def write_calibration(self, cal):
        from lerobot.motors import MotorCalibration
        self.bus.write_calibration({n: MotorCalibration(**{k: int(v[k]) for k in (
            'id', 'drive_mode', 'homing_offset', 'range_min', 'range_max')})
            for n, v in cal.items()})

    def torque_off(self):
        self.bus.disable_torque(num_retry=3)

    def position_mode(self):
        from lerobot.motors.feetech import OperatingMode
        for n in self.names:
            self.bus.write('Operating_Mode', n, OperatingMode.POSITION.value)

    def half_turn_homings(self):
        return {k: int(v) for k, v in self.bus.set_half_turn_homings().items()}

    def close(self):
        try:
            self.bus.disconnect(disable_torque=False)   # leave torque exactly as it was
        except Exception:  # noqa: BLE001
            pass


class SimBus:
    """Simulated SO-101 motor bus, state kept in a JSON file (RAWBOT_HW_SIM)."""

    def __init__(self, port, motors, baudrate=None):
        self.file = os.environ['RAWBOT_HW_SIM']
        self.port = port
        self.names = [m[0] for m in motors]
        self.ids = {m[0]: int(m[1]) for m in motors}
        if not os.path.exists(port):
            raise ConnectionError("Could not connect on port '%s'." % port)
        self.fh = open(port, 'rb')   # hold it like a real serial port would
        self.baudrate = 1000000
        self.t0 = now()
        st = self._dev()
        if st.get('baudrate_mismatch'):
            self.baudrate = 115200

    def _all(self):
        with open(self.file) as f:
            return json.load(f)

    def _dev(self):
        d = self._all().get('ports', {}).get(self.port)
        if d is None or d.get('unplugged') or not os.path.exists(self.port):
            raise ConnectionError('Port %s: no status packet (device unplugged?)' % self.port)
        return d

    def _store(self, dev):
        data = self._all()
        data['ports'][self.port] = dev
        tmp = self.file + '.tmp%d' % os.getpid()
        with open(tmp, 'w') as f:
            json.dump(data, f)
        os.replace(tmp, self.file)

    def ping(self):
        d = self._dev()
        if self.baudrate != 1000000:
            return {}
        return {int(k): int(v) for k, v in d.get('motors', {}).items()
                if int(k) not in d.get('fail_ids', [])}

    def firmware(self, ids):
        return {int(i): '3.10' for i in ids}

    def _raw(self, d, mid):
        """Present position = actual - homing offset (Feetech)."""
        base = d.get('actual', {}).get(str(mid), 2048)
        if d.get('wiggle') or d.get('sweep'):
            amp = 1500 if d.get('sweep') else 25
            base = base + int(amp * math.sin((now() - self.t0) * (1.3 + 0.2 * mid)))
        cal = d.get('calibration', {}).get(str(mid), {})
        return (base - int(cal.get('homing_offset', 0))) % RESOLUTION

    def read(self, data, name):
        d = self._dev()
        mid = self.ids[name]
        if str(mid) not in d.get('motors', {}) or mid in d.get('fail_ids', []):
            raise ConnectionError('[TxRxResult] There is no status packet!')
        cal = d.get('calibration', {}).get(str(mid), {})
        if data == 'Present_Position':
            return self._raw(d, mid)
        if data == 'Torque_Enable':
            return int(d.get('torque', {}).get(str(mid), 1))
        if data == 'Homing_Offset':
            return int(cal.get('homing_offset', 0))
        if data == 'Min_Position_Limit':
            return int(cal.get('range_min', 0))
        if data == 'Max_Position_Limit':
            return int(cal.get('range_max', RESOLUTION - 1))
        raise KeyError(data)

    def sync_pos(self, names):
        return {n: self.read('Present_Position', n) for n in names}

    def write_calibration(self, cal):
        d = self._dev()
        for n, v in cal.items():
            d.setdefault('calibration', {})[str(self.ids[n])] = {
                k: int(v[k]) for k in ('homing_offset', 'range_min', 'range_max')}
        d['writes'] = int(d.get('writes', 0)) + 1
        self._store(d)

    def torque_off(self):
        d = self._dev()
        d['torque'] = {str(i): 0 for i in self.ids.values()}
        self._store(d)

    def position_mode(self):
        self._dev()

    def half_turn_homings(self):
        d = self._dev()
        out = {}
        for n, mid in self.ids.items():
            actual = d.get('actual', {}).get(str(mid), 2048)
            out[n] = int(actual - (RESOLUTION - 1) // 2)
            d.setdefault('calibration', {})[str(mid)] = {
                'homing_offset': out[n], 'range_min': 0, 'range_max': RESOLUTION - 1}
        d['writes'] = int(d.get('writes', 0)) + 1
        if d.get('sweep_on_calibrate'):
            d['sweep'] = True
        self._store(d)
        return out

    def close(self):
        try:
            self.fh.close()
        except OSError:
            pass


def open_bus(cfg):
    cls = SimBus if os.environ.get('RAWBOT_HW_SIM') else LeRobotBus
    return cls(cfg['port'], cfg['motors'], cfg.get('baudrate'))


# ------------------------------------------------------------------ probe ---
def model_name(number):
    return {777: 'sts3215'}.get(int(number), 'model %s' % number)


def read_calibration(bus, names):
    cal, errors = {}, {}
    for n in names:
        try:
            cal[n] = {'id': bus.ids[n], 'drive_mode': 0,
                      'homing_offset': bus.read('Homing_Offset', n),
                      'range_min': bus.read('Min_Position_Limit', n),
                      'range_max': bus.read('Max_Position_Limit', n)}
        except Exception as e:  # noqa: BLE001
            errors[n] = str(e)[:160]
    return cal, errors


def calibration_state(cal, names):
    if not cal or len(cal) < len(names):
        return 'unknown'
    factory = all(c['homing_offset'] == 0 and c['range_min'] == 0 and
                  c['range_max'] == RESOLUTION - 1 for c in cal.values())
    return 'factory' if factory else 'calibrated'


def probe(cfg):
    out = {'ok': False, 'port': cfg['port'], 'at': now(), 'expected': [m[1] for m in
                                                                       cfg['motors']]}
    try:
        bus = open_bus(cfg)
    except Exception as e:  # noqa: BLE001
        out['error'] = 'Could not open %s: %s' % (cfg['port'], str(e).strip()[:240])
        return out
    try:
        out['baudrate'] = bus.baudrate
        found = bus.ping()
        fw = bus.firmware(found) if found else {}
        out['found'] = {str(i): {'model_number': mn, 'model': model_name(mn),
                                 'firmware': fw.get(i)} for i, mn in sorted(found.items())}
        expected = {m[1]: m for m in cfg['motors']}
        out['missing_ids'] = sorted(i for i in expected if i not in found)
        out['unexpected_ids'] = sorted(i for i in found if i not in expected)
        out['wrong_model_ids'] = sorted(i for i in found if i in expected and
                                        model_name(found[i]) != expected[i][2])
        joints = {}
        for name, mid, _model in cfg['motors']:
            j = {'id': mid, 'comm': 'fail'}
            if mid in found:
                try:
                    j['position'] = bus.read('Present_Position', name)
                    j['torque'] = bool(bus.read('Torque_Enable', name))
                    j['comm'] = 'ok'
                except Exception as e:  # noqa: BLE001
                    j['error'] = str(e)[:160]
            else:
                j['error'] = 'motor %d did not answer' % mid
            joints[name] = j
        out['joints'] = joints
        ok_names = [n for n, j in joints.items() if j['comm'] == 'ok']
        cal, cal_err = read_calibration(bus, ok_names)
        out['calibration'] = cal
        out['calibration_errors'] = cal_err
        out['calibration_state'] = calibration_state(cal, [m[0] for m in cfg['motors']])
        out['ok'] = not out['missing_ids'] and not out['wrong_model_ids'] and \
            all(j['comm'] == 'ok' for j in joints.values())
        if not found:
            out['error'] = ('No motor answered on %s at %d baud. Check the power supply of the '
                            'arm and the cable to the motors.' % (cfg['port'], bus.baudrate))
        elif not out['ok']:
            out['error'] = 'Motor scan incomplete: %s.' % '; '.join(filter(None, [
                out['missing_ids'] and 'no answer from ID %s' % ', '.join(
                    map(str, out['missing_ids'])),
                out['wrong_model_ids'] and 'unexpected motor model on ID %s' % ', '.join(
                    map(str, out['wrong_model_ids'])),
                [n for n, j in joints.items() if j['comm'] != 'ok' and j['id'] in found] and
                'reads failed on %s' % ', '.join(n for n, j in joints.items()
                                                  if j['comm'] != 'ok' and j['id'] in found)]))
    except Exception as e:  # noqa: BLE001
        out['error'] = 'Motor scan failed: %s' % str(e).strip()[:240]
    finally:
        bus.close()
    return out


# ---------------------------------------------------------------- monitor ---
def monitor(cfg):
    st_path, ctl_path = cfg['status_file'], cfg['control_file']
    names = [m[0] for m in cfg['motors']]
    st = {'phase': 'connecting', 'pid': os.getpid(), 'port': cfg['port'], 'names': names}
    write_status(st_path, st)
    try:
        bus = open_bus(cfg)
    except Exception as e:  # noqa: BLE001
        write_status(st_path, dict(st, phase='failed', error=str(e)[:240]))
        return 2
    fails, last_ok, n_ok, t_rate = 0, now(), 0, now()
    period = 1.0 / float(cfg.get('rate_hz', 20))
    rate = 0.0
    parent = os.getppid()
    try:
        while True:
            if read_control(ctl_path).get('command') == 'stop' or os.getppid() != parent:
                # stopped, or the training service is gone: give the port back
                write_status(st_path, dict(st, phase='stopped'))
                return 0
            t = now()
            try:
                pos = bus.sync_pos(names)
                fails = 0
                last_ok = t
                n_ok += 1
                if t - t_rate >= 1.0:
                    rate, n_ok, t_rate = n_ok / (t - t_rate), 0, t
                st.update(phase='running', t=t, positions=pos, ok={n: True for n in names},
                          rate_hz=round(rate, 1), error=None)
            except Exception as e:  # noqa: BLE001
                fails += 1
                st.update(error=str(e)[:200], ok={n: False for n in names})
                if t - last_ok > float(cfg.get('lost_after_s', 1.0)):
                    write_status(st_path, dict(st, phase='lost', t=t,
                                               error='Lost contact with the motors: %s' % st[
                                                   'error']))
                    return 3
            write_status(st_path, st)
            time.sleep(max(0.0, period - (now() - t)))
    finally:
        bus.close()


# ------------------------------------------------------------- calibration ---
def calibrate(cfg):
    st_path, ctl_path = cfg['status_file'], cfg['control_file']
    names = [m[0] for m in cfg['motors']]
    hb_timeout = float(cfg.get('heartbeat_timeout_s', 15))
    st = {'phase': 'connecting', 'pid': os.getpid(), 'mode': cfg.get('mode', 'calibrate'),
          'names': names, 'written': False}

    def push(**kw):
        st.update(kw)
        write_status(st_path, st)

    push()
    try:
        bus = open_bus(cfg)
    except Exception as e:  # noqa: BLE001
        push(phase='failed', error='Could not open %s: %s' % (cfg['port'], str(e)[:200]))
        return 2
    original = None

    def restore(reason):
        if st['written'] and original:
            try:
                bus.write_calibration(original)
                push(restored=True)
            except Exception as e:  # noqa: BLE001
                push(restore_error='Could not write the original calibration back: %s' % e)
        push(phase='cancelled', end_reason=reason)

    try:
        found = bus.ping()
        missing = [m[1] for m in cfg['motors'] if m[1] not in found]
        if missing:
            push(phase='failed', error='No answer from motor ID %s; calibration needs all six.'
                 % ', '.join(map(str, missing)))
            return 2
        original, errs = read_calibration(bus, names)
        if errs:
            push(phase='failed', error='Could not read the current calibration: %s' % errs)
            return 2
        push(phase='ready', original=original)
        last_cmd_t = None
        mins = maxes = None
        parent = os.getppid()
        while True:
            if os.getppid() != parent:
                restore('The training service stopped.')
                return 0
            ctl = read_control(ctl_path)
            hb = float(ctl.get('heartbeat') or 0)
            cmd, ct = ctl.get('command'), ctl.get('t')
            if st['phase'] not in ('ready',) and hb and now() - hb > hb_timeout:
                restore('The page stopped sending its heartbeat (closed or offline).')
                return 0
            fresh = ct is not None and ct != last_cmd_t
            if fresh:
                last_cmd_t = ct
            if fresh and cmd == 'cancel':
                restore('Cancelled by the operator.')
                return 0
            ph = st['phase']
            if fresh and cmd == 'begin' and ph == 'ready':
                bus.torque_off()
                bus.position_mode()
                if st['mode'] == 'apply':
                    prof = cfg['profile']
                    st['written'] = True
                    bus.write_calibration(prof)
                    back, _ = read_calibration(bus, names)
                    same = all(back.get(n, {}).get(k) == int(prof[n][k]) for n in names
                               for k in ('homing_offset', 'range_min', 'range_max'))
                    if not same:
                        restore('The motors did not keep the profile values.')
                        return 2
                    push(phase='done', result=back)
                    return 0
                push(phase='torque_off')
            elif fresh and cmd == 'set_middle' and ph == 'torque_off':
                st['written'] = True
                homings = bus.half_turn_homings()
                pos = bus.sync_pos(names)
                mins, maxes = dict(pos), dict(pos)
                push(phase='recording', homings=homings, positions=pos, mins=mins, maxes=maxes)
            elif fresh and cmd == 'finish' and ph == 'recording':
                rng = [n for n in names if n != FULL_TURN]
                flat = [n for n in rng if mins[n] == maxes[n]]
                small = [n for n in rng if maxes[n] - mins[n] < 100]
                if flat or small:
                    push(warning='Move %s through its whole range before finishing.' %
                         ', '.join(flat or small))
                    continue
                final = {}
                for n in names:
                    final[n] = {'id': bus.ids[n], 'drive_mode': 0,
                                'homing_offset': st['homings'][n],
                                'range_min': 0 if n == FULL_TURN else mins[n],
                                'range_max': RESOLUTION - 1 if n == FULL_TURN else maxes[n]}
                bus.write_calibration(final)
                back, _ = read_calibration(bus, names)
                same = all(back.get(n, {}).get(k) == final[n][k] for n in names
                           for k in ('homing_offset', 'range_min', 'range_max'))
                if not same:
                    restore('The motors did not keep the new calibration values.')
                    return 2
                push(phase='done', result=final, warning=None)
                return 0
            if st['phase'] == 'recording':
                try:
                    pos = bus.sync_pos(names)
                    for n in names:
                        mins[n] = min(mins[n], pos[n])
                        maxes[n] = max(maxes[n], pos[n])
                    push(positions=pos, mins=mins, maxes=maxes, t=now())
                except Exception as e:  # noqa: BLE001
                    restore('Lost contact with the motors during calibration: %s' % e)
                    return 3
            time.sleep(0.03)
    except Exception as e:  # noqa: BLE001
        try:
            restore('Error: %s' % e)
        finally:
            push(phase='failed', error=str(e)[:240])
        return 2
    finally:
        bus.close()


def ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return {'ok': False, 'error': 'pyserial is not installed in this environment'}
    out = []
    for p in list_ports.comports():
        if p.vid is None:
            continue
        out.append({'device': p.device, 'name': os.path.basename(p.device),
                    'vid': '%04x' % p.vid, 'pid': '%04x' % (p.pid or 0),
                    'serial': p.serial_number, 'manufacturer': p.manufacturer,
                    'product': p.product, 'usb_path': p.location, 'by_id': None})
    return {'ok': True, 'ports': out}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    cmd = sys.argv[1]
    if cmd == 'ports':
        print(OUT + json.dumps(ports()), flush=True)
        return 0
    cfg = json.load(open(sys.argv[2]))
    if cmd == 'probe':
        print(OUT + json.dumps(probe(cfg)), flush=True)
        return 0
    if cmd == 'monitor':
        return monitor(cfg)
    if cmd == 'calibrate':
        return calibrate(cfg)
    print('unknown command %s' % cmd, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
