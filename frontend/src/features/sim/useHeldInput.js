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

// Keyboard and gamepad as "hold to move" inputs: each joint gets a velocity in -1..1 while a
// key or stick is held. onTick(velocities) is called every 100 ms while anything is held and
// once with {} when everything is released - the receiver (simulated robot, or the
// supervised follower jog) stops on its own when the ticks stop.

import { useEffect, useRef, useState } from 'react';

export const KEY_PAIRS = [
  ['q', 'a'],
  ['w', 's'],
  ['e', 'd'],
  ['r', 'f'],
  ['t', 'g'],
  ['y', 'h'],
  ['u', 'j'],
  ['i', 'k'],
  ['o', 'l'],
];
// gamepad (standard mapping): sticks for the first four joints, bumpers and triggers for
// the last two
export const PAD_MAP = ['Left stick ←→', 'Left stick ↑↓', 'Right stick ←→', 'Right stick ↑↓', 'LB / RB', 'LT / RT'];
const DEAD = 0.15;

const typing = (el) => el && (/^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName) || el.isContentEditable);

export default function useHeldInput({ names, keyboard, gamepad, onTick }) {
  const keys = useRef(new Set());
  const tickRef = useRef(onTick);
  tickRef.current = onTick;
  const namesRef = useRef(names);
  namesRef.current = names;
  const [pad, setPad] = useState('');
  const [held, setHeld] = useState({});
  const last = useRef('{}');

  useEffect(() => {
    if (!keyboard) return undefined;
    const set = keys.current;
    const down = (e) => {
      if (typing(document.activeElement) || e.ctrlKey || e.metaKey || e.altKey) return;
      const k = e.key.toLowerCase();
      if (KEY_PAIRS.some((p) => p.includes(k))) {
        set.add(k);
        e.preventDefault();
      }
    };
    const up = (e) => set.delete(e.key.toLowerCase());
    const blur = () => set.clear();
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', blur);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', blur);
      set.clear();
    };
  }, [keyboard]);

  useEffect(() => {
    if (!keyboard && !gamepad) return undefined;
    const id = setInterval(() => {
      const names = namesRef.current || [];
      const v = {};
      if (keyboard) {
        KEY_PAIRS.forEach(([plus, minus], i) => {
          if (!names[i]) return;
          const d = (keys.current.has(plus) ? 1 : 0) - (keys.current.has(minus) ? 1 : 0);
          if (d) v[names[i]] = d;
        });
      }
      if (gamepad && navigator.getGamepads) {
        const gp = Array.from(navigator.getGamepads() || []).find(Boolean);
        setPad(gp ? gp.id : '');
        if (gp) {
          const ax = (i, inv) => {
            const a = (gp.axes[i] || 0) * (inv ? -1 : 1);
            return Math.abs(a) < DEAD ? 0 : Math.round(a * 100) / 100;
          };
          const btn = (i) => (gp.buttons[i] ? gp.buttons[i].value || (gp.buttons[i].pressed ? 1 : 0) : 0);
          const vals = [ax(0), ax(1, true), ax(2), ax(3, true), btn(5) - btn(4), btn(7) - btn(6)];
          vals.forEach((x, i) => {
            if (names[i] && Math.abs(x) >= DEAD) v[names[i]] = Math.max(-1, Math.min(1, x));
          });
        }
      }
      const s = JSON.stringify(v);
      if (s !== '{}' || last.current !== '{}') {
        tickRef.current(v);
        if (s !== last.current) setHeld(v);
      }
      last.current = s;
    }, 100);
    return () => {
      clearInterval(id);
      if (last.current !== '{}') tickRef.current({});
      last.current = '{}';
    };
  }, [keyboard, gamepad]);

  return { held, pad };
}
