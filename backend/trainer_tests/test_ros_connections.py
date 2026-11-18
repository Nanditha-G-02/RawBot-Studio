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
