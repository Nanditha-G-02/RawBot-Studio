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

import { useSelector } from 'react-redux';

// '' when the feature may start, otherwise the reason it cannot. Unknown (host service not
// reachable, no status yet, another robot) never blocks: the page behaves as before.
export default function useHardwareGate(feature) {
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const { status, offline } = useSelector((s) => s.hardware);
  if (offline || !status || status.robot_type !== robotType) return '';
  // the ROS topics of the arms are not known yet (robot runtime not reached): don't guess
  if (status.mode !== 'simulation' && !Object.keys(status.topics || {}).length) return '';
  const cap = feature === 'recording' ? status.recording_gate : status.capabilities?.[feature];
  if (!cap || cap.available) return '';
  return cap.reason || 'Not available with the connected hardware.';
}

export function useHardwareStatus() {
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const { status, offline } = useSelector((s) => s.hardware);
  return !offline && status && status.robot_type === robotType ? status : null;
}
