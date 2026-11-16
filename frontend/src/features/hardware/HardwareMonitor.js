// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Mounted once (App.js). Keeps the hardware status of the active robot fresh so every page
// can enable or explain its features, tells the host service which ROS topics carry the
// arms, keeps physical_ai_server's action source in line with the hardware configuration,
// and toasts connection changes (unplugged, lost, reconnected, ...).

import { useCallback, useEffect, useRef } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import toast from 'react-hot-toast';
import rosConnectionManager from '../../utils/rosConnectionManager';
import { useRosLink } from '../../hooks/useConnectionMonitor';
import { usePlanningService } from '../../hooks/usePlanningService';
import { hwActionSource, hwSetup, hwStatus, topicsFromParams } from './hardwareApi';
import { setHardwareOffline, setHardwareStatus } from './hardwareSlice';

const SHOW = new Set(['lost', 'unplug', 'plug', 'reconnect', 'estop', 'calibration', 'mode']);

export default function HardwareMonitor() {
  const dispatch = useDispatch();
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const status = useSelector((s) => s.hardware.status);
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const linked = useRosLink(2000);
  const { call } = usePlanningService();
  const lastSeq = useRef(null);
  const topicsSent = useRef('');
  const sourceSent = useRef('');

  const refresh = useCallback(async () => {
    if (!robotType) return;
    try {
      const st = await hwStatus(robotType);
      dispatch(setHardwareStatus(st));
      if (lastSeq.current === null) {
        lastSeq.current = st.event_seq;
      } else if (st.event_seq > lastSeq.current) {
        (st.events || [])
          .filter((e) => e.id > lastSeq.current && SHOW.has(e.kind) && (e.kind !== 'mode' || e.tone === 'ok'))
          .forEach((e) => {
            const opts = { id: `hw-${e.id}`, duration: 6000 };
            if (e.tone === 'ok') toast.success(e.text, opts);
            else if (e.tone === 'err') toast.error(e.text, opts);
            else toast(e.text, { ...opts, icon: e.tone === 'warn' ? '⚠' : 'ℹ' });
          });
        lastSeq.current = st.event_seq;
      }
    } catch (e) {
      dispatch(setHardwareOffline(e.message));
    }
  }, [robotType, dispatch]);

  useEffect(() => {
    lastSeq.current = null;
    if (!robotType) return undefined;
    refresh();
    const id = setInterval(refresh, 2000);
    const on = () => refresh();
    window.addEventListener('rawbot-hardware-refresh', on);
    return () => {
      clearInterval(id);
      window.removeEventListener('rawbot-hardware-refresh', on);
    };
  }, [robotType, refresh]);

  // which ROS topics carry the follower state and the leader (command) - from the robot
  // configuration of physical_ai_server, the same place recording and rollout read them
  useEffect(() => {
    if (!robotType || !linked || !status || status.robot_type !== robotType) return;
    let cancelled = false;
    call('robot.profile', {})
      .then((res) => {
        if (cancelled) return;
        const topics = topicsFromParams(res.params);
        const key = `${robotType}|${JSON.stringify(topics)}`;
        if (!Object.keys(topics).length || topicsSent.current === key) return;
        topicsSent.current = key;
        if (JSON.stringify(topics) !== JSON.stringify(status.topics || {})) {
          hwSetup(robotType, { topics }).then((st) => dispatch(setHardwareStatus(st))).catch(() => {});
        }
      })
      .catch(() => {});
    return () => {
      cancelled = true;
    };
  }, [robotType, linked, status?.robot_type, status?.topics, call, dispatch]); // eslint-disable-line react-hooks/exhaustive-deps

  // Follower Only records hand-guided demonstrations: physical_ai_server then takes the
  // action from the follower. Every other configuration keeps the leader (the original).
  const mode = status?.robot_type === robotType ? status?.mode : null;
  useEffect(() => {
    if (!robotType || !linked || !mode) return;
    const want = mode === 'follower_only' ? 'follower' : 'leader';
    const key = `${robotType}|${want}`;
    if (sourceSent.current === key) return;
    sourceSent.current = key;
    (async () => {
      let ok = null;
      try {
        const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
        ok = await new Promise((resolve) => {
          const srv = new ROSLIB.Service({
            ros,
            name: '/physical_ai_server/set_parameters',
            serviceType: 'rcl_interfaces/srv/SetParameters',
          });
          const t = setTimeout(() => resolve(null), 4000);
          srv.callService(
            new ROSLIB.ServiceRequest({ parameters: [{ name: 'action_source', value: { type: 4, string_value: want } }] }),
            (r) => {
              clearTimeout(t);
              resolve(!!(r && r.results && r.results[0] && r.results[0].successful));
            },
            () => {
              clearTimeout(t);
              resolve(false);
            }
          );
        });
      } catch (e) {
        ok = null;
      }
      if (ok === null) sourceSent.current = ''; // try again later
      hwActionSource(robotType, ok).catch(() => {});
    })();
  }, [robotType, linked, mode, rosbridgeUrl]);

  return null;
}

export const refreshHardware = () => window.dispatchEvent(new Event('rawbot-hardware-refresh'));
