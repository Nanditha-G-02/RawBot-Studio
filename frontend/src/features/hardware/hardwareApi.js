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

// Flexible robot setup on the host service (/train-api/hardware, hardware/manager.py).

import { trainerFetch } from '../../hooks/useTrainer';

const q = (rt) => `robot_type=${encodeURIComponent(rt || '')}`;
const post = (path, robotType, body = {}, timeoutMs) =>
  trainerFetch(`/hardware/${path}`, { method: 'POST', body: { robot_type: robotType, ...body }, timeoutMs });

export const hwStatus = (rt) => trainerFetch(`/hardware/status?${q(rt)}`, { timeoutMs: 8000 });
export const hwSetup = (rt, body) => post('setup', rt, body);
export const hwDetect = (rt) => post('detect', rt, {}, 90000);
export const hwAssign = (rt, key, role, label) => post('assign', rt, { key, role, label }, 60000);
export const hwUnassign = (rt, role) => post('unassign', rt, { role });
export const hwConfigure = (rt, role, body) => post('configure', rt, { role, ...body });
export const hwConnect = (rt, role) => post('connect', rt, { role }, 60000);
export const hwDisconnect = (rt, role) => post('disconnect', rt, { role });
export const hwEstop = (source = 'page') => post('estop', '', { source });
export const hwActionSource = (rt, supported, source) => post('action-source', rt, { supported, source });
// the simulated robot (hardware/simbot.py) and the Planning choices (hardware/planning.py)
export const hwSim = (rt, body) => post('sim', rt, body, 6000);
export const hwPlanning = (rt, body) => post('setup', rt, body);

export const calStart = (rt, role, mode = 'calibrate', profileId) =>
  post('calibration', rt, { role, mode, profile_id: profileId }, 30000);
export const calCommand = (id, command, name) =>
  trainerFetch(`/hardware/calibration/${id}/command`, { method: 'POST', body: { command, name }, timeoutMs: 15000 });
export const calDismiss = () => post('calibration/dismiss', '');

export const profiles = (rt, role) => trainerFetch(`/hardware/profiles?${q(rt)}${role ? `&role=${role}` : ''}`);
export const profileRename = (id, name) => trainerFetch(`/hardware/profiles/${id}/rename`, { method: 'POST', body: { name } });
export const profileDelete = (id) => trainerFetch(`/hardware/profiles/${id}/delete`, { method: 'POST', body: {} });
export const profileInstall = (id, lerobotId) =>
  trainerFetch(`/hardware/profiles/${id}/install`, { method: 'POST', body: { lerobot_id: lerobotId } });

export const jogLimits = (dataset) => trainerFetch(`/hardware/jog/limits?dataset=${encodeURIComponent(dataset)}`, { timeoutMs: 300000 });
export const jogPrepare = (rt, request) => post('jog/prepare', rt, { request });
export const jogStart = (rt, request, token, confirmations) => post('jog', rt, { request, token, confirmations });
export const jogCommand = (id, body) =>
  trainerFetch(`/hardware/jog/${id}/command`, { method: 'POST', body, timeoutMs: 4000 });

export const captures = (rt) => trainerFetch(`/hardware/captures?${q(rt)}`);
export const saveCapture = (rt, body) => post('captures', rt, body, 60000);

// joint_topic_list entries look like "follower:/joint_states"
export function topicsFromParams(params) {
  const out = {};
  (params?.joint_topic_list || []).forEach((e) => {
    const i = e.indexOf(':');
    if (i < 0) return;
    const k = e.slice(0, i).trim().toLowerCase();
    const t = e.slice(i + 1).trim();
    if (k.startsWith('follower') && !out.follower) out.follower = t;
    else if (k.startsWith('leader') && !out.leader) out.leader = t;
  });
  return out;
}

// camera_topic_list entries look like "global:/camera/color/image_raw/compressed"
export function camerasFromParams(params) {
  const out = {};
  (params?.camera_topic_list || []).forEach((e) => {
    const i = e.indexOf(':');
    if (i > 0) out[e.slice(0, i).trim()] = e.slice(i + 1).trim();
  });
  return out;
}

export function jointsFromParams(params) {
  const jo = params?.joint_order || {};
  return jo.leader || Object.values(jo)[0] || [];
}

export const MODE_OPTIONS = [
  { value: 'leader_follower', label: 'Leader + Follower', short: 'L + F' },
  { value: 'leader_only', label: 'Leader Only', short: 'Leader' },
  { value: 'leader_sim', label: 'Physical Leader + Simulated Follower', short: 'L + Sim' },
  { value: 'follower_only', label: 'Follower Only', short: 'Follower' },
  { value: 'simulation', label: 'Simulation Only', short: 'Sim' },
];

export const STATE_LABEL = {
  connected: 'Connected',
  connecting: 'Connecting…',
  calibrating: 'Calibrating',
  detected: 'Detected',
  in_use: 'Port in use',
  lost: 'Connection lost',
  not_found: 'Not plugged in',
  not_configured: 'Not set up',
  disconnected: 'Disconnected',
  error: 'Check failed',
  unused: 'Not used',
};

export const stateTone = (s) =>
  s === 'connected' ? 'ok' : s === 'lost' || s === 'error' ? 'err' : s === 'in_use' || s === 'calibrating' || s === 'connecting' ? 'warn' : s === 'detected' ? 'sky' : 'idle';
