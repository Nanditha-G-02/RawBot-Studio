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

import React, { useEffect, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import {
  MdBackHand,
  MdCancel,
  MdCheckCircle,
  MdAutoAwesome,
  MdKeyboard,
  MdPrecisionManufacturing,
  MdReplay,
  MdSmartToy,
  MdSportsEsports,
  MdTimeline,
  MdTune,
  MdVideocam,
  MdWarning,
} from 'react-icons/md';
import { StatusDot, Toggle } from '../../components/ui';
import { hwHybrid, hwPlanning, hwSim, rolloutCommand, rolloutHeartbeat } from '../hardware/hardwareApi';
import { setHardwareStatus } from '../hardware/hardwareSlice';
import { useHardwareStatus } from '../hardware/useHardwareGate';
import { JogPanel } from '../hardware/HardwareSetup';
import SimControl from '../sim/SimControl';
import SimViewport from '../sim/SimViewport';
import { SIM_TOPIC } from '../sim/simTopics';

const MANUAL = ['gui', 'keyboard', 'gamepad'];
const SIM_LEADS = [...MANUAL, 'script', 'replay'];

const ROBOT_TONE = { connected: 'ok', running: 'sky', lost: 'err', error: 'err', unused: 'idle', off: 'idle' };

// short labels and icons; the full label and any reason are in the tooltip
const SHORT = {
  action: {
    leader: ['Leader', MdPrecisionManufacturing],
    hand_guided: ['Hand', MdBackHand],
    gui: ['GUI', MdTune],
    keyboard: ['Keys', MdKeyboard],
    gamepad: ['Pad', MdSportsEsports],
    script: ['Script', MdTimeline],
    replay: ['Replay', MdReplay],
    policy: ['Policy', MdAutoAwesome],
  },
  target: { follower: ['Real', MdPrecisionManufacturing], sim: ['Sim', MdSmartToy] },
  observation: { follower: ['Real', MdPrecisionManufacturing], sim: ['Sim', MdSmartToy] },
};
// reasons that will not change with this robot and configuration: the option is hidden
// (temporary ones - an arm not connected, simulation off - stay visible, disabled)
const NEVER = /^(> \d+ joints|joints unknown|simulated follower|physical only|simulation only|no leader|no follower|needs Follower Only|needs Leader \+ Follower)$/;

function robotLabel(r) {
  if (r.kind === 'simulated') return r.state === 'running' ? 'Running' : 'Off';
  if (!r.used) return 'Not used';
  if (r.state === 'connected') return r.source === 'serial' ? 'Serial' : 'Connected';
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
    <div className="flex flex-wrap items-center gap-x-2 gap-y-1.5">
      <div className="label w-[52px] shrink-0">{label}</div>
      <div className="flex flex-wrap gap-1" role="radiogroup" aria-label={label} data-plan-choice={kind}>
        {options.map((o) => {
          const on = o.id === value;
          const [short, Icon] = SHORT[kind][o.id] || [o.label, null];
          const hide = !o.available && !on && NEVER.test(o.short || '');
          return (
            <button
              key={o.id}
              type="button"
              role="radio"
              aria-checked={on}
              hidden={hide}
              disabled={!o.available || busy}
              title={o.available ? o.label : `${o.label}: ${o.reason}`}
              onClick={() => !on && onPick(o.id)}
              data-option={o.id}
              data-available={o.available ? 'yes' : 'no'}
              className={clsx(
                hide ? 'hidden' : 'inline-flex',
                'items-center gap-1.5 rounded-pill border px-2.5 py-1 text-[12px] leading-tight transition-colors',
                on && o.available && 'bg-ink text-paper border-ink',
                on && !o.available && 'border-signal-warn bg-signal-warnbg',
                !on && o.available && 'border-rule bg-paper hover:border-ink',
                !o.available && !on && 'border-rule text-ink-4 cursor-not-allowed'
              )}
            >
              {Icon && <Icon className="text-[14px] shrink-0" aria-hidden />}
              <span>{short}</span>
              {!o.available && <span className="text-[10.5px] text-ink-4">· {o.short}</span>}
            </button>
          );
        })}
      </div>
    </div>
  );
}

const CHECK_ICON = { ok: MdCheckCircle, warn: MdWarning, err: MdCancel };
const CHECK_TONE = { ok: 'text-signal-ok', warn: 'text-signal-warn', err: 'text-signal-err' };

function Checks({ checks }) {
  if (!checks || !checks.length) return null;
  return (
    <div className="flex flex-wrap gap-x-3 gap-y-1 text-[12px]" data-plan-checks>
      {checks.map((c) => {
        const Icon = CHECK_ICON[c.state] || MdWarning;
        return (
          <span key={c.id} className="inline-flex items-center gap-1" title={c.detail} data-plan-check={c.id} data-check-state={c.state}>
            <Icon className={clsx('text-[14px]', CHECK_TONE[c.state])} aria-hidden />
            {c.label}
          </span>
        );
      })}
    </div>
  );
}

// scene: the objects and targets of the selected plan episode (sim/sceneItems.js) - shown in
// the 3D view and sent to the simulated robot, so its cameras and MuJoCo have them too
export default function PlanningSetup({ compact = false, scene = null }) {
  const dispatch = useDispatch();
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const sceneJson = scene && scene.length ? JSON.stringify(scene) : '';
  useEffect(() => {
    if (!sceneJson || !robotType) return;
    hwSim(robotType, { command: 'scene', items: JSON.parse(sceneJson), label: 'planning' }).catch(() => {});
  }, [sceneJson, robotType]);
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
  const needs = !!pl && (!pl.record.available || pl.selection.target === 'sim' || SIM_LEADS.includes(pl.selection.action) || pl.selection.action === 'policy');
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
        <StatusDot tone="idle" label={offline ? 'Hardware status unavailable' : 'Reading the hardware…'} />
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
  const leaderSim = p.config === 'leader_sim';
  const simLeads = p.config === 'sim_leader';
  const robot = p.robot || {};
  const camsLive = p.cameras.filter((c) => c.live !== false).length;
  const recTitle = [rec.reason, ...(rec.streams || []).map((x) => `· ${x.label}`)].filter(Boolean).join('\n');

  const summary = (
    <div className="flex flex-wrap items-center gap-x-4 gap-y-1.5 min-w-0">
      <span className="chip-ink" data-plan-config={p.config} data-plan-auto={p.auto ? 'yes' : 'no'} title={p.auto ? 'Only a physical leader: its simulated follower is the target' : undefined}>
        {p.config_label}
        {p.auto && <span className="ml-1 opacity-70">· auto</span>}
      </span>
      {robot.dof > 0 && (
        <span className="inline-flex items-center gap-1 text-[12px] text-ink-3" data-plan-robot-model title={`${robot.source}: ${robot.names.join(', ')}`}>
          <MdPrecisionManufacturing className="text-[14px]" aria-hidden />
          {robot.dof} joints{robot.gripper ? ' · gripper' : ''}
        </span>
      )}
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
      {p.cameras.length > 0 && (
        <span className="inline-flex items-center gap-1 text-[12px]" title={p.cameras.map((c) => `${c.name}: ${c.topic}`).join('\n')}>
          <MdVideocam className={clsx('text-[14px]', camsLive === p.cameras.length ? 'text-signal-ok' : 'text-signal-err')} aria-hidden />
          {p.cameras.map((c) => (
            <span
              key={c.name}
              data-plan-camera={c.name}
              data-camera-live={c.live === false ? 'no' : 'yes'}
              data-camera-sim={c.sim ? 'yes' : undefined}
              className={clsx(c.live === false ? 'text-signal-err' : 'text-ink-3')}
            >
              {c.name}
              {c.sim && <span className="text-ink-4"> · sim</span>}
            </span>
          ))}
        </span>
      )}
      <span className="flex-1" />
      <span data-plan-record={rec.available ? 'ready' : 'blocked'} className="text-[12px]" title={recTitle}>
        <StatusDot tone={rec.available ? 'ok' : 'warn'} label={rec.available ? `Ready · ${selectionText(p)}` : `${selectionText(p)} · not ready`} />
      </span>
    </div>
  );

  const control = <ControlFor robotType={robotType} status={status} p={p} sim={sim} compact={compact} />;

  if (compact) {
    return (
      <div className="w-full border-b border-rule bg-white px-3 py-3 space-y-2" data-plan-setup="compact" data-plan-selection={`${sel.action}|${sel.target}|${sel.observation}`}>
        <div className="flex items-center gap-2 text-[12px]">
          <span className="label">Control</span>
          <span className="font-medium truncate">{selectionText(p)}</span>
        </div>
        <Checks checks={p.checks} />
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
          <div className="space-y-2 min-w-0">
            <Choice label="Source" kind="action" options={p.action_sources} value={sel.action} onPick={pick('action')} busy={busy} />
            <Choice label="Target" kind="target" options={p.targets} value={sel.target} onPick={pick('target')} busy={busy} />
            <Choice label="State" kind="observation" options={p.observations} value={sel.observation} onPick={pick('observation')} busy={busy} />
            <div className="flex flex-wrap items-center gap-x-4 gap-y-1.5 pt-1">
              {!simMode && !leaderSim && !simLeads && (
                <span data-plan-sim-twin={sim.twin ? 'on' : 'off'} title="Run a simulated twin next to the physical robot">
                  <Toggle checked={!!sim.twin} disabled={busy} onChange={(v) => save({ sim_twin: v })} label={<span className="text-[12px]">Simulated twin</span>} />
                </span>
              )}
              <Checks checks={p.checks} />
            </div>
            {!rec.available && (
              <div className="text-[12px] text-signal-warn" data-plan-record-reason>
                {rec.reason}
              </div>
            )}
          </div>
          <div className="min-w-0">{control}</div>
          {(sel.target === 'sim' || sim.twin) && sim.running !== false && (
            <div className="xl:col-span-2" data-plan-sim-view>
              <SimViewport robotType={robotType} topic={sim.state_topic || SIM_TOPIC} chain={status.sim?.chain} items={scene || undefined} />
            </div>
          )}
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
    body = <Note>Move the leader; the follower follows.</Note>;
  } else if (action === 'hand_guided') {
    body = <Note>Move the follower by hand.</Note>;
  } else if (action === 'policy') {
    body = <PolicyControl robotType={robotType} p={p} sim={sim} />;
  } else if (SIM_LEADS.includes(action)) {
    // the simulated robot is the leader: the input moves it, the real follower follows it
    body = (
      <>
        <SimControl robotType={robotType} sim={sim} action={action} compact />
        <JogPanel robotType={robotType} status={status} cap={status.capabilities.follower_control} follow />
      </>
    );
  }
  return (
    <div className="space-y-3">
      {body}
      {sim.running && !SIM_LEADS.includes(action) && <Note>Twin mirrors the follower.</Note>}
      {!compact && !SIM_LEADS.includes(action) && <Compare compare={p.compare} />}
    </div>
  );
}

// a trained policy drives the follower (host rollout); the operator can take over with the
// simulated leader and hand back - the rollout worker stays the one command owner. This page
// keeps the rollout's heartbeat while it is open, so recording can run from here.
function PolicyControl({ robotType, p, sim }) {
  const dispatch = useDispatch();
  const ro = p.rollout;
  const [busy, setBusy] = useState(false);
  const id = ro?.id;
  useEffect(() => {
    if (!id) return undefined;
    const iv = setInterval(() => rolloutHeartbeat(id).catch(() => {}), 500);
    return () => clearInterval(iv);
  }, [id]);
  if (!ro) return <Note>Load a policy on Inference → Host rollout.</Note>;
  const act = async (fn) => {
    setBusy(true);
    try {
      const st = await fn();
      if (st && st.planning) dispatch(setHardwareStatus(st));
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  const running = ro.status === 'running';
  const n = (ro.interventions || []).length + (ro.op_since ? 1 : 0);
  return (
    <div className="space-y-2" data-policy={ro.status} data-operator={ro.operator ? 'on' : 'off'}>
      <div className="flex flex-wrap items-center gap-2 text-[12px]">
        <StatusDot tone={running ? 'warn' : 'idle'} pulse={running} label={ro.operator ? 'Operator in control' : `Policy · ${ro.status}`} />
        {n > 0 && <span className="chip num">{n} takeover{n === 1 ? '' : 's'}</span>}
        <span className="flex-1" />
        {ro.status === 'ready' || ro.status === 'paused' ? (
          <button className="btn-primary btn-sm" disabled={busy} onClick={() => act(() => rolloutCommand(id, 'run'))} data-action="policy-run">
            ▶ Run
          </button>
        ) : (
          <button className="btn btn-sm" disabled={busy || !running} onClick={() => act(() => rolloutCommand(id, 'pause'))} data-action="policy-pause">
            ❚❚ Pause
          </button>
        )}
        {running && (
          <button
            className={ro.operator ? 'btn-primary btn-sm' : 'btn btn-sm'}
            disabled={busy}
            onClick={() => act(() => hwHybrid(robotType, !ro.operator))}
            data-action="policy-takeover"
          >
            {ro.operator ? 'Hand back' : 'Take over'}
          </button>
        )}
      </div>
      {ro.workspace && <div className="text-[12px] text-signal-warn">Held: tool {ro.workspace}</div>}
      {ro.operator && <SimControl robotType={robotType} sim={sim} action="gui" compact />}
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
