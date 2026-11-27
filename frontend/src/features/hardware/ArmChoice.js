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

// Which arms are real and which are simulated: one choice per role instead of a list of named
// configurations. Any robot, any combination; the hardware configuration (mode) follows from it.

import React from 'react';
import clsx from 'clsx';
import { modeFor, rolesOf } from './hardwareApi';

const ROLES = [
  ['leader', 'Leader'],
  ['follower', 'Follower'],
];
const KINDS = [
  ['real', 'Real'],
  ['sim', 'Sim'],
];

export default function ArmChoice({ mode, onChange, disabled, variant = 'app' }) {
  const roles = rolesOf(mode);
  const home = variant === 'home';
  const pick = (role, kind) => {
    const next = { ...roles, [role]: kind };
    const m = modeFor(next.leader, next.follower);
    if (m !== mode) onChange(m);
  };
  return (
    <div className="grid gap-1" data-mode-choice={mode || ''} role="group" aria-label="Arms">
      {ROLES.map(([role, label]) => (
        <div key={role} className="flex items-center gap-2" role="radiogroup" aria-label={label}>
          <span className={clsx('w-[62px] shrink-0', home ? 'text-[11.5px] text-[#6B7079]' : 'label')}>{label}</span>
          <div className={clsx('inline-flex gap-1', home ? 'rounded-[12px] bg-[#EEF0F3] p-1' : 'rounded-pill bg-paper-2 p-[3px]')}>
            {KINDS.map(([kind, text]) => {
              const on = roles[role] === kind;
              return (
                <button
                  key={kind}
                  type="button"
                  role="radio"
                  aria-checked={on}
                  disabled={disabled || !mode}
                  data-arm={role}
                  data-choice={kind}
                  onClick={() => pick(role, kind)}
                  className={clsx(
                    'px-3 py-1 text-[12px] font-medium transition-all disabled:opacity-40',
                    home ? 'rounded-[9px]' : 'rounded-pill',
                    home
                      ? on
                        ? 'bg-white text-[#0B0C0E] shadow-[0_1px_2px_rgba(11,12,14,0.12)]'
                        : 'text-[#6B7079] hover:text-[#0B0C0E]'
                      : on
                      ? 'bg-ink text-white shadow-sm'
                      : 'text-ink-2 hover:text-ink',
                    !mode && 'opacity-60'
                  )}
                >
                  {text}
                </button>
              );
            })}
          </div>
        </div>
      ))}
    </div>
  );
}
