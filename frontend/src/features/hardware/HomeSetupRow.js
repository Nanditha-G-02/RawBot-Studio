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

// Home, after a robot is confirmed: which arms this setup has (Leader + Follower, Leader
// Only, Follower Only, Simulation Only) and the state of each arm. The full setup (detect,
// connect, calibrate, jog) is on Robots → Hardware.

import React, { useState } from 'react';
import { useDispatch } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { MODE_OPTIONS, STATE_LABEL, hwSetup } from './hardwareApi';
import { setHardwareStatus } from './hardwareSlice';
import { useHardwareStatus } from './useHardwareGate';

const DOT = { ok: 'bg-signal-ok', err: 'bg-signal-err', warn: 'bg-signal-warn', sky: 'bg-blue-500', idle: 'bg-[#C9CDD3]' };
const tone = (s) =>
  s === 'connected' ? 'ok' : s === 'lost' || s === 'error' ? 'err' : ['in_use', 'calibrating', 'connecting'].includes(s) ? 'warn' : s === 'detected' ? 'sky' : 'idle';

export default function HomeSetupRow({ robotType, onOpen, disabled }) {
  const dispatch = useDispatch();
  const status = useHardwareStatus();
  const [saving, setSaving] = useState('');
  if (!robotType) return null;

  const choose = async (mode) => {
    if (!status || mode === status.mode || saving) return;
    setSaving(mode);
    try {
      dispatch(setHardwareStatus(await hwSetup(robotType, { mode })));
    } catch (e) {
      toast.error(e.message);
    } finally {
      setSaving('');
    }
  };

  return (
    <div className="space-y-2" data-home-hardware>
      <div className="flex items-center justify-between gap-2">
        <div className="label text-[9.5px] tracking-[0.12em]">Hardware setup</div>
        <button type="button" className="lg-btn lg-btn-sm" onClick={onOpen} data-action="open-hardware">
          Detect &amp; connect →
        </button>
      </div>
      <div className="grid grid-cols-4 gap-1 rounded-[14px] bg-[#EEF0F3] p-1" role="radiogroup" aria-label="Available hardware">
        {MODE_OPTIONS.map((m) => {
          const on = status?.mode === m.value;
          return (
            <button
              key={m.value}
              type="button"
              role="radio"
              aria-checked={on}
              data-home-mode={m.value}
              disabled={!status || disabled || !!saving}
              onClick={() => choose(m.value)}
              title={m.label}
              className={clsx(
                'rounded-[10px] px-1.5 py-1.5 text-[11.5px] font-medium leading-tight transition-all',
                on ? 'bg-white text-[#0B0C0E] shadow-[0_1px_2px_rgba(11,12,14,0.12)]' : 'text-[#6B7079] hover:text-[#0B0C0E]',
                !status && 'opacity-60'
              )}
            >
              {m.label}
            </button>
          );
        })}
      </div>
      {status ? (
        <div className="flex flex-wrap gap-x-4 gap-y-1 text-[11.5px] text-[#6B7079]">
          {['leader', 'follower'].map((role) => {
            const card = status.arms?.[role];
            if (!card) return null;
            return (
              <span key={role} className="inline-flex items-center gap-1.5" data-home-arm={role}>
                <span className={clsx('h-1.5 w-1.5 rounded-full', DOT[tone(card.state)])} />
                {role === 'leader' ? 'Leader' : 'Follower'}: {STATE_LABEL[card.state] || card.state}
              </span>
            );
          })}
        </div>
      ) : (
        <div className="text-[11.5px] text-[#6B7079]">
          Hardware status needs the training service (./start_physios.sh). Until then everything works as before.
        </div>
      )}
    </div>
  );
}
