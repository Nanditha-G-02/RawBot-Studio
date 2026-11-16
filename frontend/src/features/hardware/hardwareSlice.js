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

// Latest hardware status of the active robot (polled by HardwareMonitor). `offline` means the
// host service is not reachable: features are then NOT blocked (the original behaviour).

import { createSlice } from '@reduxjs/toolkit';

const hardwareSlice = createSlice({
  name: 'hardware',
  initialState: { status: null, robotType: '', offline: false, error: '', updatedAt: 0 },
  reducers: {
    setHardwareStatus(state, action) {
      state.status = action.payload;
      state.robotType = action.payload?.robot_type || '';
      state.offline = false;
      state.error = '';
      state.updatedAt = Date.now();
    },
    setHardwareOffline(state, action) {
      state.offline = true;
      state.error = action.payload || '';
    },
  },
});

export const { setHardwareStatus, setHardwareOffline } = hardwareSlice.actions;
export default hardwareSlice.reducer;
