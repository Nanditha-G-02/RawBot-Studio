// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0

// The scene objects with their current poses, from the simulated robot's objects topic
// (backend simbot.OBJECTS_TOPIC): planned positions with the kinematic backend, the physics
// result with MuJoCo. A ref that is read every frame (no re-render per message).

import { useEffect, useRef } from 'react';
import { useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import rosConnectionManager from '../../utils/rosConnectionManager';

export const OBJECTS_TOPIC = '/rawbot/sim/objects';

export function subscribeObjects(ros, onItems) {
  const sub = new ROSLIB.Topic({ ros, name: OBJECTS_TOPIC, messageType: 'std_msgs/msg/String', throttle_rate: 30 });
  sub.subscribe((m) => {
    try {
      const d = JSON.parse(m.data);
      onItems(d.items || [], d);
    } catch (e) {
      // a malformed message is skipped
    }
  });
  return sub;
}

export default function useSimObjects(enabled = true) {
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const ref = useRef({ items: [], backend: null, stamp: 0 });
  useEffect(() => {
    if (!enabled) return undefined;
    let sub = null;
    let alive = true;
    rosConnectionManager
      .getConnection(rosbridgeUrl)
      .then((ros) => {
        if (!alive) return;
        sub = subscribeObjects(ros, (items, d) => {
          ref.current = { items, backend: d.backend, stamp: performance.now() };
        });
      })
      .catch(() => {});
    return () => {
      alive = false;
      if (sub) sub.unsubscribe();
    };
  }, [rosbridgeUrl, enabled]);
  return ref;
}
