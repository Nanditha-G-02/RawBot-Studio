# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0

"""Publishes an arm's joint states on ROS when RawBot Studio reads the arm itself.

Without bridge.py nothing publishes the follower state topic (e.g. /follower_joint_states), so
monitoring, the simulation view and hand-guided recording see no follower. When RawBot
Studio's read-only serial session has the follower and no other node publishes that topic,
this relays the session's readings (converted to radians, manager.ticks_to_rad) as
sensor_msgs/JointState at 30 Hz. It publishes state only - never a command - and stops when
the session ends, the readings go stale or another publisher (bridge.py) appears.
"""

import threading
import time

from physios_trainer.hardware.simbot import joint_state

RATE_HZ = 30.0
STALE_S = 0.5


class StateRelay:
    def __init__(self, url, bridge_factory=None, log=print, thread=True):
        self.url = url
        self.bridge_factory = bridge_factory
        self.log = log
        self.lock = threading.Lock()
        self.topic = None
        self.sampler = None       # () -> (age, names, values, extra)
        self.br = None
        self.advertised = set()
        self.published = 0
        self.error = None
        self.last = None
        self._stop = False
        if thread:
            threading.Thread(target=self._loop, daemon=True).start()

    def set(self, topic, sampler):
        with self.lock:
            if topic != self.topic:
                self.published = 0
            self.topic, self.sampler = topic, sampler

    def clear(self):
        with self.lock:
            self.topic, self.sampler, self.last = None, None, None

    def owns(self, topic):
        return bool(topic and topic == self.topic)

    def tick(self):
        """One relay step; returns the message published, or None."""
        with self.lock:
            topic, sampler = self.topic, self.sampler
        if not topic or not sampler:
            return None
        age, names, vals, _ = sampler()
        if age is None or age > STALE_S or not names:
            return None
        msg = joint_state(names, vals, [])
        if self.bridge_factory is None:
            self.last = msg
            return msg
        try:
            if self.br is None or not getattr(self.br, 'alive', True):
                self.br = self.bridge_factory(self.url)
                self.advertised = set()
            if topic not in self.advertised:
                self.br.advertise(topic, 'sensor_msgs/msg/JointState')
                self.advertised.add(topic)
            self.br.publish(topic, msg)
            self.published += 1
            self.error = None
            self.last = msg
        except Exception as e:  # noqa: BLE001
            self.error = 'Cannot publish over rosbridge (%s): %s' % (self.url, str(e)[:160])
            self.br = None
        return msg

    def _loop(self):
        period = 1.0 / RATE_HZ
        while not self._stop:
            t = time.time()
            self.tick()
            time.sleep(1.0 if self.error else max(0.0, period - (time.time() - t)))

    def stop(self):
        self._stop = True

    def view(self):
        return {'topic': self.topic, 'published': self.published, 'error': self.error} \
            if self.topic else None
