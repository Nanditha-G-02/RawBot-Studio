# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""rosbridge connections are closed whenever they are replaced or fail, so a flaky rosbridge
cannot collect stale clients until it runs out of file handles (Errno 24)."""

import socket
import threading
import time

from physios_trainer import rollout_worker as rw
from physios_trainer.hardware import relay as relaymod


def _server(reply, keep_open=False):
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(4)
    conns = []

    def run():
        while True:
            try:
                c, _ = srv.accept()
            except OSError:
                return
            conns.append(c)
            c.recv(4096)
            c.sendall(reply)
            if not keep_open:
                c.close()
    threading.Thread(target=run, daemon=True).start()
    return srv, conns


def test_refused_handshake_closes_the_socket(monkeypatch):
    srv, _ = _server(b'HTTP/1.1 400 Bad Request\r\n\r\n')
    made = []
    real = socket.create_connection

    def spy(*a, **k):
        s = real(*a, **k)
        made.append(s)
        return s
    monkeypatch.setattr(rw.socket, 'create_connection', spy)
    try:
        rw.WS('ws://127.0.0.1:%d' % srv.getsockname()[1])
        raise AssertionError('handshake should fail')
    except ConnectionError:
        pass
    assert made and made[0].fileno() == -1
    srv.close()


def test_bridge_closes_itself_when_rosbridge_drops_it():
    ok = (b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n'
          b'Connection: Upgrade\r\n\r\n')
    srv, _ = _server(ok)            # handshake, then the server closes
    br = rw.Bridge('ws://127.0.0.1:%d' % srv.getsockname()[1])
    for _ in range(50):
        if not br.alive:
            break
        time.sleep(0.02)
    assert not br.alive and br.ws.sock.fileno() == -1
    srv.close()


class FakeBridge:
    made = []

    def __init__(self, url, fail=False):
        self.closed = False
        self.alive = True
        self.fail = fail
        FakeBridge.made.append(self)

    def advertise(self, *a):
        pass

    def publish(self, *a):
        if self.fail:
            raise ConnectionError('rosbridge gone')

    def close(self):
        self.closed = True


def test_relay_closes_a_failed_connection_before_reconnecting():
    FakeBridge.made = []
    fails = iter([True, False])
    r = relaymod.StateRelay('ws://x', bridge_factory=lambda u: FakeBridge(u, next(fails)),
                            thread=False)
    r.set('/follower_joint_states', lambda: (0.0, ['a'], [0.1], None))
    r.tick()
    assert r.error and FakeBridge.made[0].closed
    r.tick()
    assert r.error is None and len(FakeBridge.made) == 2 and not FakeBridge.made[1].closed
    # a connection that died by itself is closed too before it is replaced
    FakeBridge.made[1].alive = False
    fails = iter([False])
    r.tick()
    assert FakeBridge.made[1].closed and len(FakeBridge.made) == 3


def test_websocket_masking_needs_no_numpy(monkeypatch):
    # the training service runs on a plain Python; numpy must not be needed to talk to rosbridge
    import sys
    monkeypatch.setitem(sys.modules, 'numpy', None)      # any 'import numpy' now fails
    for data in (b'', b'a', b'abcd', b'{"op": "subscribe"}' * 50, bytes(range(256)) * 300):
        mask = b'\x01\x80\x7f\xff'
        out = rw._mask_fast(data, mask)
        assert out == bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        assert rw._mask_fast(out, mask) == data


def test_hardware_status_path_imports_without_numpy():
    # run the service's ROS clients in a child Python where numpy cannot be imported
    import subprocess
    import sys
    code = ('import sys; sys.modules["numpy"] = None\n'
            'from physios_trainer.rollout_worker import WS, Bridge, drop, _mask_fast\n'
            'from physios_trainer.hardware import manager, relay, simbot, kinematics\n'
            'assert _mask_fast(b"xyz", b"abcd") == bytes([0x19, 0x1b, 0x19])\n'
            'print("ok")\n')
    import os
    r = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                       cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    assert r.returncode == 0 and r.stdout.strip() == 'ok', r.stderr[-800:]


def test_server_config_without_cameras_has_no_empty_lists():
    # ROS 2 cannot load an empty list parameter: one such file stopped the whole robot server
    import yaml
    from physios_trainer.robots.readiness import server_config_yaml
    cfg = {'cameras': [], 'joint_state_topic': '/ufactory/joint_states', 'leader_topic': '',
           'namespace': '', 'joints': [{'ros_joint': 'joint%d' % i, 'enabled': True}
                                       for i in range(1, 7)]}
    key, text = server_config_yaml({'id': 'lite6', 'name': 'Lite6'}, cfg)
    d = yaml.safe_load(text)['physical_ai_server']['ros__parameters'][key]
    assert d['camera_topic_list'] == [''] and d['rosbag_extra_topic_list'] == ['']
    assert all(v not in (None, []) for v in d.values())


def test_roswatch_subscribes_with_the_real_topic_type(monkeypatch):
    """A leader topic is guessed to be a JointTrajectory (bridge.py); an arm driver that
    publishes JointState there is re-subscribed with its real type (rosapi)."""
    from physios_trainer.hardware import manager as m

    class Br:
        made = []

        def __init__(self, url):
            self.alive = True
            self.latest = {}
            self.subs = []
            self.types = {}
            Br.made.append(self)

        def subscribe(self, t, typ, throttle_ms=0):
            self.subs.append((t, typ))

        def unsubscribe(self, t):
            self.subs.append((t, None))

        def call(self, service, args=None, timeout=3.0):
            if service == '/rosapi/topic_type':
                return {'type': self.types.get(args['topic'], '')}
            return {'publishers': []}

        def close(self):
            self.alive = False
    monkeypatch.setattr(rw, 'Bridge', Br)
    w = m.RosWatch('ws://x')
    w.watch({'/ufactory/joint_states': 'trajectory_msgs/msg/JointTrajectory'})
    for _ in range(40):
        if Br.made and Br.made[0].subs:
            break
        time.sleep(0.05)
    br = Br.made[0]
    assert br.subs[0] == ('/ufactory/joint_states', 'trajectory_msgs/msg/JointTrajectory')
    br.types['/ufactory/joint_states'] = 'sensor_msgs/msg/JointState'   # the driver appears
    for _ in range(80):
        if ('/ufactory/joint_states', 'sensor_msgs/msg/JointState') in br.subs:
            break
        time.sleep(0.05)
    w._stop = True
    assert ('/ufactory/joint_states', None) in br.subs
    assert ('/ufactory/joint_states', 'sensor_msgs/msg/JointState') in br.subs
    br.latest['/ufactory/joint_states'] = (time.time(), {'name': ['joint1'], 'position': [0.1]})
    assert w.kind('/ufactory/joint_states') == 'state'
