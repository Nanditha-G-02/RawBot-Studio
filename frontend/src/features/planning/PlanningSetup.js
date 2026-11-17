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

// "Robot & control" for Planning and Record: what is connected (physical arms, the simulated
// robot, cameras), and three independent choices - action source, target robot, observation -
// with only the compatible ones enabled (hardware/planning.py decides, from the live hardware
// status). Below them, the control for the current choice.

import React, { useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import { StatusDot, Toggle } from '../../components/ui';
import { hwPlanning } from '../hardware/hardwareApi';
import { setHardwareStatus } from '../hardware/hardwareSlice';
import { useHardwareStatus } from '../hardware/useHardwareGate';
import { JogPanel } from '../hardware/HardwareSetup';
import SimControl from '../sim/SimControl';

const MANUAL = ['gui', 'keyboard', 'gamepad'];

const ROBOT_TONE = { connected: 'ok', running: 'sky', lost: 'err', error: 'err', unused: 'idle', off: 'idle' };

function robotLabel(r) {
  if (r.kind === 'simulated') return r.state === 'running' ? 'Running' : 'Off';
  if (!r.used) return 'Not used';
  if (r.state === 'connected') return r.source === 'serial' ? 'Read-only session' : 'Connected';
  return (r.state || 'unknown').replace(/_/g, ' ');
}

export function selectionText(p) {
  if (!p) return '';
  const a = p.action_sources.find((o) => o.id === p.selection.action);
  const t = p.targets.find((o) => o.id === p.selection.target);
  return `${a?.label || p.selection.action} → ${t?.label || p.selection.target}`;
}

function Choice({ label, kind, options, value, onPick, busy }) {
  return (
    <div className="grid gap-1.5 md:grid-cols-[120px_1fr] md:items-center">
      <div className="label">{label}</div>
      <div className="flex flex-wrap gap-1.5" role="radiogroup" aria-label={label} data-plan-choice={kind}>
        {options.map((o) => {
          const on = o.id === value;
          return (
            <button
              key={o.id}
              type="button"
              role="radio"
              aria-checked={on}
              disabled={!o.available || busy}
              title={o.available ? o.label : o.reason}
              onClick={() => !on && onPick(o.id)}
              data-option={o.id}
              data-available={o.available ? 'yes' : 'no'}
              className={clsx(
                'rounded-pill border px-2.5 py-1 text-[12px] leading-tight transition-colors text-left',
                on && o.available && 'bg-ink text-paper border-ink',
                on && !o.available && 'border-signal-warn bg-signal-warnbg',
                !on && o.available && 'border-rule bg-paper hover:border-ink',
                !o.available && !on && 'border-rule text-ink-4 cursor-not-allowed'
              )}
            >
              {o.label}
              {!o.available && <span className="block text-[10.5px] text-ink-4">{o.short}</span>}
            </button>
          );
        })}
      </div>
    </div>
  );
}

export default function PlanningSetup({ compact = false }) {
  const dispatch = useDispatch();
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const offline = useSelector((s) => s.hardware.offline);
  const status = useHardwareStatus();
  const [busy, setBusy] = useState(false);
  // the operator's own open / closed choice wins; otherwise open only when something needs
  // attention (not ready to record) or operating (simulated robot, direct control)
  const [stored, setStored] = useState(() => {
    try {
      return window.localStorage.getItem('rawbot.plan.setup.open');
    } catch (e) {
      return null;
    }
  });
  const pl = status?.planning;
  const needs = !!pl && (!pl.record.available || pl.selection.target === 'sim' || MANUAL.includes(pl.selection.action));
  const open = stored === null ? needs : stored === '1';
  const toggleOpen = () => {
    const v = open ? '0' : '1';
    setStored(v);
    try {
      window.localStorage.setItem('rawbot.plan.setup.open', v);
    } catch (e) {
      /* per-browser convenience only */
    }
  };

  if (!robotType) return null;
  const p = status?.planning;
  if (!p) {
    if (compact) return null;
    return (
      <div className="border-b border-rule bg-paper px-5 py-2 text-[12px] text-ink-3" data-plan-setup="unknown">
        <StatusDot tone="idle" label={offline ? 'Hardware status unavailable (host service not reachable)' : 'Reading the hardware…'} />
      </div>
    );
  }

  const save = async (body) => {
    setBusy(true);
    try {
      dispatch(setHardwareStatus(await hwPlanning(robotType, body)));
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  const pick = (kind) => (id) => save({ selection: { ...p.selection, [kind]: id } });
  const sel = p.selection;
  const rec = p.record;
  const sim = status.sim || p.sim || {};
  const simMode = status.mode === 'simulation';
  const leaderSim = status.mode === 'leader_sim';

  const summary = (
    <div className="flex flex-wrap items-center gap-x-4 gap-y-1.5 min-w-0">
      <span className="chip-ink" data-plan-config={p.config}>
        {p.config_label}
      </span>
      {p.robots
        .filter((r) => !(leaderSim && r.id === 'follower') && (r.used || r.kind === 'physical' || !simMode))
        .map((r) => (
          <span key={r.id} data-plan-robot={r.id} data-robot-state={r.state || 'unknown'} title={r.joints.length ? `${r.joints.length} joints: ${r.joints.join(', ')}` : undefined}>
            <StatusDot
              tone={!r.used && r.kind === 'physical' ? 'idle' : ROBOT_TONE[r.state] || 'warn'}
              pulse={r.kind === 'simulated' && r.state === 'running' && sim.drive !== 'manual'}
              label={
                <span>
                  {r.label} <span className="text-ink-3">· {robotLabel(r)}</span>
                </span>
              }
            />
          </span>
        ))}
      {p.cameras.map((c) => (
        <span key={c.name} data-plan-camera={c.name} title={c.topic}>
          <StatusDot tone={c.live === false ? 'err' : c.live ? 'ok' : 'idle'} label={<span className="text-ink-3">{c.name}</span>} />
        </span>
      ))}
      <span className="flex-1" />
      <span data-plan-record={rec.available ? 'ready' : 'blocked'} className="text-[12px]" title={rec.reason || undefined}>
        <StatusDot tone={rec.available ? 'ok' : 'warn'} label={rec.available ? `Ready to record · ${selectionText(p)}` : `${selectionText(p)} · not ready`} />
      </span>
    </div>
  );

  const control = (
    <ControlFor robotType={robotType} status={status} p={p} sim={sim} compact={compact} />
  );

  if (compact) {
    return (
      <div className="w-full border-b border-rule bg-white px-3 py-3 space-y-2" data-plan-setup="compact" data-plan-selection={`${sel.action}|${sel.target}|${sel.observation}`}>
        <div className="flex items-center gap-2 text-[12px]">
          <span className="label">Control</span>
          <span className="font-medium truncate">{selectionText(p)}</span>
        </div>
        {!rec.available && <div className="text-[11.5px] text-signal-warn">{rec.reason}</div>}
        {control}
      </div>
    );
  }

  return (
    <section className="border-b border-ink bg-white" data-plan-setup="full" data-plan-selection={`${sel.action}|${sel.target}|${sel.observation}`}>
      <div className="flex items-start gap-3 px-5 py-2.5">
        <button type="button" onClick={toggleOpen} className="label shrink-0 pt-1 hover:text-ink" aria-expanded={open} data-action="plan-setup-toggle">
          {open ? '▾' : '▸'} Robot &amp; control
        </button>
        <div className="flex-1 min-w-0">{summary}</div>
      </div>
      {open && (
        <div className="grid gap-4 border-t border-rule px-5 py-3 xl:grid-cols-[minmax(0,1fr)_minmax(0,1fr)]">
          <div className="space-y-2.5 min-w-0">
            <Choice label="Action source" kind="action" options={p.action_sources} value={sel.action} onPick={pick('action')} busy={busy} />
            <Choice label="Target robot" kind="target" options={p.targets} value={sel.target} onPick={pick('target')} busy={busy} />
            <Choice label="Observation" kind="observation" options={p.observations} value={sel.observation} onPick={pick('observation')} busy={busy} />
            <div className="flex flex-wrap items-center gap-x-4 gap-y-1.5 pt-1">
              {!simMode && !leaderSim && (
                <span data-plan-sim-twin={sim.twin ? 'on' : 'off'}>
                  <Toggle checked={!!sim.twin} disabled={busy} onChange={(v) => save({ sim_twin: v })} label={<span className="text-[12px]">Simulated twin</span>} />
                </span>
              )}
              <details className="text-[12px]">
                <summary className="cursor-pointer text-ink-3 hover:text-ink">What is recorded</summary>
                <ul className="mt-1.5 space-y-0.5" data-plan-streams>
                  {rec.streams.map((s) => (
                    <li key={s.id} className="flex gap-2">
                      <span className="text-ink-4">·</span>
                      <span>{s.label}</span>
                    </li>
                  ))}
                </ul>
              </details>
            </div>
            {!rec.available && (
              <div className="text-[12px] text-signal-warn" data-plan-record-reason>
                {rec.reason}
              </div>
            )}
          </div>
          <div className="min-w-0">{control}</div>
        </div>
      )}
    </section>
  );
}

// the control for the current choice
function ControlFor({ robotType, status, p, sim, compact }) {
  const { action, target } = p.selection;
  const a = p.action_sources.find((o) => o.id === action);
  if (!a?.available) return null;
  if (target === 'sim') {
    return (
      <div className="space-y-3">
        <SimControl robotType={robotType} sim={sim} action={action} compact={compact} />
        {!compact && <Compare compare={p.compare} />}
      </div>
    );
  }
  // the physical follower is the target
  let body = null;
  if (action === 'leader') {
    body = <Note>Move the leader arm; the follower follows it (bridge.py teleoperation).</Note>;
  } else if (action === 'hand_guided') {
    body = <Note>Move the follower by hand. Its joints are recorded as both state and action.</Note>;
  } else if (MANUAL.includes(action)) {
    body = (
      <JogPanel
        robotType={robotType}
        status={status}
        cap={status.capabilities.follower_control}
        input={action}
      />
    );
  }
  return (
    <div className="space-y-3">
      {body}
      {sim.running && <Note>The simulated robot mirrors the physical follower.</Note>}
      {!compact && <Compare compare={p.compare} />}
    </div>
  );
}

function Note({ children }) {
  return <p className="text-[12px] text-ink-3">{children}</p>;
}

function Compare({ compare }) {
  if (!compare) return null;
  return (
    <div data-plan-compare>
      <div className="flex items-center gap-2 mb-1">
        <span className="label">Real vs simulation</span>
        <span className={clsx('chip num', compare.max_abs > 0.1 ? 'chip-warn' : 'chip-ok')}>
          max Δ {compare.max_abs.toFixed(3)} · {compare.worst}
        </span>
      </div>
      <table className="w-full text-[11.5px] font-mono">
        <thead>
          <tr className="text-left text-ink-4">
            <th className="font-normal">joint</th>
            <th className="font-normal text-right">real</th>
            <th className="font-normal text-right">sim</th>
            <th className="font-normal text-right">Δ</th>
          </tr>
        </thead>
        <tbody>
          {compare.joints.map((j) => (
            <tr key={j.name} className="border-t border-rule">
              <td className="truncate max-w-[120px]">{j.name}</td>
              <td className="text-right tabular-nums">{j.real.toFixed(3)}</td>
              <td className="text-right tabular-nums">{j.sim.toFixed(3)}</td>
              <td className={clsx('text-right tabular-nums', Math.abs(j.delta) > 0.1 && 'text-signal-warn')}>{j.delta.toFixed(3)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
