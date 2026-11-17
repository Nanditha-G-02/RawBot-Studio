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

// Hardware setup of the active robot (Robots → Hardware): configuration (Leader + Follower,
// Leader Only, Follower Only, Simulation Only), one card per arm, unassigned USB devices,
// what the connected hardware allows, calibration, direct follower control and leader capture.
//
// Nothing on this page moves the arm except the jog panel, and that only after a reviewed,
// confirmed session. Detect, Connect and the monitors only read from the motors.

import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import toast from 'react-hot-toast';
import clsx from 'clsx';
import { Callout, Field, KeyValue, Modal, Segmented, Select, Sheet, Spinner, StatusDot, TextInput } from '../../components/ui';
import rosConnectionManager from '../../utils/rosConnectionManager';
import { usePlanningService } from '../../hooks/usePlanningService';
import { trainerFetch } from '../../hooks/useTrainer';
import * as api from './hardwareApi';
import { MODE_OPTIONS, STATE_LABEL, stateTone } from './hardwareApi';
import { setHardwareStatus } from './hardwareSlice';
import { refreshHardware } from './HardwareMonitor';
import useHeldInput from '../sim/useHeldInput';

const ROLE_NAME = { leader: 'Leader arm', follower: 'Follower arm' };
const fmt = (v, d = 3) => (v === null || v === undefined || !Number.isFinite(Number(v)) ? '—' : Number(v).toFixed(d));

function useAction(robotType, onStatus) {
  const [busy, setBusy] = useState('');
  const run = useCallback(
    async (label, fn, okText) => {
      setBusy(label);
      try {
        const st = await fn();
        if (st && st.arms) onStatus(st);
        if (okText) toast.success(okText);
        return st;
      } catch (e) {
        toast.error(e.message, { duration: 7000 });
        return null;
      } finally {
        setBusy('');
        refreshHardware();
      }
    },
    [onStatus]
  );
  return [busy, run];
}

// ------------------------------------------------------------------ live joints ---
function LiveJoints({ robotType, card, rosbridgeUrl }) {
  const [vals, setVals] = useState(null);
  const [age, setAge] = useState(null);
  const range = useRef({});
  const lastT = useRef(0);
  const source = card.live?.source;
  useEffect(() => {
    range.current = {};
    if (card.state !== 'connected') return undefined;
    let alive = true;
    let cleanup = () => {};
    if (source === 'ros' && card.topic) {
      rosConnectionManager
        .getConnection(rosbridgeUrl)
        .then((ros) => {
          if (!alive) return;
          const leader = card.role === 'leader';
          const topic = new ROSLIB.Topic({
            ros,
            name: card.topic,
            messageType: leader ? 'trajectory_msgs/msg/JointTrajectory' : 'sensor_msgs/msg/JointState',
            throttle_rate: 100,
          });
          const cb = (m) => {
            const names = leader ? m.joint_names || [] : m.name || [];
            const pos = leader ? (m.points && m.points[0] ? m.points[0].positions : []) : m.position || [];
            const v = {};
            names.forEach((n, i) => {
              v[n] = pos[i];
            });
            lastT.current = performance.now();
            setVals(v);
          };
          topic.subscribe(cb);
          cleanup = () => topic.unsubscribe(cb);
        })
        .catch(() => {});
    } else if (source === 'serial') {
      const iv = setInterval(async () => {
        try {
          const st = await api.hwStatus(robotType);
          const c = st.arms[card.role];
          const v = {};
          Object.entries(c.joints || {}).forEach(([n, j]) => {
            v[n] = j.value;
          });
          lastT.current = performance.now();
          setVals(v);
        } catch (e) {
          /* the card shows the state */
        }
      }, 400);
      cleanup = () => clearInterval(iv);
    }
    const iv2 = setInterval(() => setAge(lastT.current ? performance.now() - lastT.current : null), 500);
    return () => {
      alive = false;
      cleanup();
      clearInterval(iv2);
    };
  }, [card.state, card.topic, card.role, source, robotType, rosbridgeUrl]);
  const entries = Object.entries(vals || {});
  entries.forEach(([n, v]) => {
    const r = range.current[n] || { lo: v, hi: v };
    range.current[n] = { lo: Math.min(r.lo, v), hi: Math.max(r.hi, v) };
  });
  return (
    <div data-live-joints={card.role}>
      <div className="label mb-1">
        Live joints {source === 'serial' ? '(motor ticks, read-only)' : `(${card.topic})`}{' '}
        {age === null ? '· waiting' : age > 1500 ? `· stale ${(age / 1000).toFixed(1)} s` : '· live'}
      </div>
      <div className="grid grid-cols-2 gap-1.5">
        {entries.map(([n, v]) => {
          const r = range.current[n];
          const pct = source === 'serial' ? (v / 4095) * 100 : r.hi > r.lo ? ((v - r.lo) / (r.hi - r.lo)) * 100 : 50;
          return (
            <div key={n} className="rounded-xl bg-paper px-2 py-1.5 text-[11.5px]">
              <div className="flex justify-between gap-2">
                <span className="font-mono truncate">{n}</span>
                <span className="font-mono tabular-nums">{source === 'serial' ? v : fmt(v)}</span>
              </div>
              <div className="h-1 mt-1 bg-paper-3 rounded-pill relative">
                <div className="absolute top-0 h-1 w-1.5 rounded-pill bg-ink" style={{ left: `${Math.max(0, Math.min(100, pct))}%` }} />
              </div>
            </div>
          );
        })}
        {!entries.length && <div className="text-[12px] text-ink-3 col-span-2">No joint data yet.</div>}
      </div>
    </div>
  );
}

// ---------------------------------------------------------------------- arm card ---
function MotorChips({ card }) {
  const m = card.motors || {};
  const found = new Set(m.found || card.device?.motor_ids || []);
  const known = m.found || card.device?.motor_ids;
  return (
    <div className="flex flex-wrap gap-1" data-motor-ids={card.role}>
      {(m.expected || []).map((id) => (
        <span
          key={id}
          className={clsx('font-mono text-[10.5px] rounded-md px-1.5 py-0.5', !known ? 'bg-paper text-ink-3' : found.has(id) ? 'bg-signal-okbg text-signal-ok' : 'bg-signal-errbg text-signal-err')}
          title={!known ? 'not scanned yet' : found.has(id) ? 'answered' : 'no answer'}
        >
          {id}
        </span>
      ))}
      {!(m.expected || []).length && <span className="text-[12px] text-ink-3">Motor scan not available for this robot type.</span>}
    </div>
  );
}

function JointComm({ card }) {
  const js = Object.entries(card.joints || {});
  if (!js.length) return <span className="text-ink-3">Not checked yet</span>;
  const bad = js.filter(([, j]) => j.comm !== 'ok');
  return (
    <span className={bad.length ? 'text-signal-warn' : 'text-signal-ok'} data-joint-comm={card.role}>
      {bad.length ? `${js.length - bad.length}/${js.length} joints answering (${bad.map(([n]) => n).join(', ')})` : `All ${js.length} joints answering`}
      {js[0][1].from_scan ? ' · last scan' : card.source === 'ros' ? ' · via ROS' : card.source === 'serial' ? ' · live' : ''}
    </span>
  );
}

function ArmCard({ robotType, card, busy, run, onConfigure, onCalibrate, rosbridgeUrl }) {
  const name = ROLE_NAME[card.role];
  const tone = stateTone(card.state);
  const dimmed = card.state === 'unused';
  const port = card.port;
  return (
    <section
      className={clsx('sheet animate-rise min-w-0', dimmed && 'opacity-60')}
      data-arm-card={card.role}
      data-arm-state={card.state}
      aria-label={name}
    >
      <header className="sheet-head">
        <div className="flex items-center gap-2 min-w-0">
          <h2 className="sheet-title truncate">{name}</h2>
          {card.device?.label && card.device.label !== `${card.role[0].toUpperCase()}${card.role.slice(1)} arm` && (
            <span className="text-[12px] text-ink-3 truncate">· {card.device.label}</span>
          )}
        </div>
        <StatusDot tone={tone} pulse={card.state === 'connected'} label={STATE_LABEL[card.state] || card.state} />
      </header>
      <div className="px-4 pb-4 pt-0.5 space-y-3">
        <p className="text-[12.5px] text-ink-2 min-h-[18px]" data-arm-detail>{card.detail}</p>
        <KeyValue
          rows={[
            ['Connection', card.state === 'connected' ? (card.source === 'ros' ? 'Verified · live ROS messages from bridge.py' : 'Verified · every motor answers (read-only)') : STATE_LABEL[card.state]],
            [
              'Serial port',
              port ? (
                <span className="font-mono text-[11.5px] break-all" data-arm-port>
                  {port.device}
                  {port.by_id ? <span className="block text-ink-3">{port.by_id.split('/').pop()}</span> : null}
                  {port.override ? <span className="chip text-[10px] ml-1">manual</span> : null}
                </span>
              ) : card.device ? (
                <span className="text-ink-3">not plugged in{card.device.last_port ? ` (last ${card.device.last_port})` : ''}</span>
              ) : (
                <span className="text-ink-3">—</span>
              ),
            ],
            card.device && [
              'Identity',
              <span className="text-[11.5px]">
                USB {card.device.vid}:{card.device.pid}
                {card.device.serial ? ` · S/N ${card.device.serial}` : ' · no serial number (follows the USB socket)'}
              </span>,
            ],
            card.holders?.length > 0 && ['Held by', <span className="font-mono text-[11.5px]">{card.holders.map((h) => `${h.name} (PID ${h.pid})`).join(', ')}</span>],
            ['Motor IDs', <MotorChips card={card} />],
            ['Calibration', <span data-arm-calibration={card.role}>{card.calibration?.text || '—'}</span>],
            ['Joints', <JointComm card={card} />],
          ]}
        />
        {card.state === 'connected' && <LiveJoints robotType={robotType} card={card} rosbridgeUrl={rosbridgeUrl} />}
        <div className="flex flex-wrap gap-2 pt-1">
          {card.can_connect && (
            <button
              className="btn-primary btn-sm"
              disabled={!!busy}
              onClick={() => run(`connect-${card.role}`, () => api.hwConnect(robotType, card.role), `${name} connected`)}
              data-action={`connect-${card.role}`}
            >
              {busy === `connect-${card.role}` && <Spinner className="border-paper border-t-transparent" />}
              {card.state === 'lost' || card.state === 'disconnected' ? 'Reconnect' : 'Connect'}
            </button>
          )}
          {card.can_disconnect && (
            <button className="btn btn-sm" disabled={!!busy} onClick={() => run(`disc-${card.role}`, () => api.hwDisconnect(robotType, card.role))} data-action={`disconnect-${card.role}`}>
              Disconnect
            </button>
          )}
          <button
            className="btn btn-sm"
            disabled={!!busy || !card.can_calibrate}
            title={card.can_calibrate ? 'Guided calibration (torque off, no motion)' : card.calibrate_reason}
            onClick={() => onCalibrate(card.role)}
            data-action={`calibrate-${card.role}`}
          >
            Calibrate
          </button>
          <button className="btn btn-sm" disabled={!!busy || dimmed} onClick={() => onConfigure(card.role)} data-action={`configure-${card.role}`}>
            Configure
          </button>
        </div>
        {!card.can_calibrate && card.calibrate_reason && !dimmed && (
          <p className="text-[11.5px] text-ink-3">Calibrate: {card.calibrate_reason}</p>
        )}
      </div>
    </section>
  );
}

// --------------------------------------------------------------- unassigned list ---
function Unassigned({ robotType, status, busy, run }) {
  const list = status.unassigned || [];
  if (!list.length) return null;
  return (
    <Sheet title="Detected USB serial devices (not assigned)" code="USB">
      <p className="text-[12.5px] text-ink-3 mb-2">
        Leader and follower motors are identical, so RawBot Studio never guesses the role from the port. Tell it which arm each
        adapter belongs to; the role is then bound to the adapter's identity (USB serial number) and checked against the motor IDs
        whenever it is plugged in.
      </p>
      <div className="grid gap-2 md:grid-cols-2" data-unassigned>
        {list.map((u) => {
          const pr = u.probe;
          return (
            <div key={u.key} className="rounded-2xl bg-paper p-3 space-y-2" data-unassigned-device={u.port.device}>
              <div className="flex items-start justify-between gap-2">
                <div className="min-w-0">
                  <div className="font-mono text-[12px] break-all">{u.port.device}</div>
                  <div className="text-[11.5px] text-ink-3">
                    {u.port.product || 'USB serial'} · {u.port.vid}:{u.port.pid}
                    {u.port.serial ? ` · S/N ${u.port.serial}` : ' · no serial number'}
                  </div>
                </div>
                {pr ? (
                  <span className={pr.ok ? 'chip-ok text-[10px]' : 'chip-warn text-[10px]'}>{pr.ok ? `${Object.keys(pr.found || {}).length} motors OK` : pr.skipped ? 'not scanned' : 'check failed'}</span>
                ) : (
                  <span className="chip text-[10px]">not scanned</span>
                )}
              </div>
              {u.registered_elsewhere && (
                <div className="text-[11.5px] text-ink-3">Registered as the {u.registered_elsewhere.role} of {u.registered_elsewhere.robot_type}.</div>
              )}
              {u.holders?.length > 0 && <div className="text-[11.5px] text-signal-warn">Held by {u.holders.map((h) => `${h.name} (PID ${h.pid})`).join(', ')} - stop it once so the motors can be checked.</div>}
              {pr && !pr.ok && pr.error && <div className="text-[11.5px] text-signal-warn">{pr.error}</div>}
              <div className="flex gap-2">
                {['leader', 'follower'].map((role) => (
                  <button
                    key={role}
                    className="btn btn-sm"
                    disabled={!!busy || (pr && !pr.ok && !pr.skipped)}
                    onClick={() => run(`assign-${u.key}`, () => api.hwAssign(robotType, u.key, role), `Registered as the ${role} arm`)}
                    data-assign={role}
                  >
                    This is the {role}
                  </button>
                ))}
              </div>
            </div>
          );
        })}
      </div>
    </Sheet>
  );
}

// ---------------------------------------------------------------- capabilities ---
const GROUPS = [
  ['leader_follower', 'With both arms'],
  ['leader', 'Leader'],
  ['follower', 'Follower'],
  ['always', 'Always'],
];
const GROUP_OF = {
  teleoperation: 'leader_follower', record: 'leader_follower', collect: 'leader_follower',
  leader_calibration: 'leader', leader_monitor: 'leader', leader_capture: 'leader',
  follower_calibration: 'follower', follower_control: 'follower', follower_monitor: 'follower', hand_guided_record: 'follower', inference: 'follower', policy_rollout: 'follower',
  simulation: 'always', model_config: 'always',
};

function Capabilities({ status }) {
  const caps = status.capabilities || {};
  return (
    <Sheet title="What you can do now" code="C">
      <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-4" data-capabilities>
        {GROUPS.map(([g, title]) => (
          <div key={g} className="space-y-1.5">
            <div className="label">{title}</div>
            {Object.entries(caps)
              .filter(([k]) => GROUP_OF[k] === g)
              .map(([k, c]) => (
                <div key={k} className="flex gap-2 text-[12.5px]" data-capability={k} data-available={c.available ? 'yes' : 'no'}>
                  <span className={clsx('shrink-0 mt-[1px] text-[11px]', c.available ? 'text-signal-ok' : 'text-ink-4')}>{c.available ? '✓' : '🔒'}</span>
                  <div className="min-w-0">
                    <div className={c.available ? 'text-ink' : 'text-ink-2'}>{c.label}</div>
                    {!c.available && c.reason && <div className="text-[11.5px] text-ink-3">{c.reason}</div>}
                  </div>
                </div>
              ))}
          </div>
        ))}
      </div>
    </Sheet>
  );
}

// ------------------------------------------------------------------ configure ---
function ConfigureModal({ robotType, role, card, onClose, onStatus }) {
  const [label, setLabel] = useState(card?.device?.label || '');
  const [port, setPort] = useState(card?.device?.port_override || '');
  const [profs, setProfs] = useState([]);
  const [lid, setLid] = useState('');
  const load = useCallback(() => api.profiles(robotType, role).then((r) => setProfs(r.profiles || [])).catch(() => {}), [robotType, role]);
  useEffect(() => {
    load();
  }, [load]);
  if (!card) return null;
  const save = async () => {
    try {
      const st = await api.hwConfigure(robotType, role, { label, port_override: port });
      onStatus(st);
      toast.success('Saved');
      onClose();
    } catch (e) {
      toast.error(e.message);
    }
  };
  return (
    <Modal open title={`Configure · ${ROLE_NAME[role]}`} onClose={onClose} width="max-w-2xl" footer={
      <>
        {card.device && (
          <button
            className="btn-ghost mr-auto text-signal-err"
            onClick={async () => {
              try {
                onStatus(await api.hwUnassign(robotType, role));
                onClose();
              } catch (e) {
                toast.error(e.message);
              }
            }}
          >
            Remove registration
          </button>
        )}
        <button className="btn" onClick={onClose}>Close</button>
        <button className="btn-primary" onClick={save} disabled={!card.device} data-action="save-arm-config">Save</button>
      </>
    }>
      <div className="space-y-4">
        {!card.device && <Callout tone="info">Assign a detected USB device to this arm first (Detect Hardware).</Callout>}
        <div className="grid gap-3 md:grid-cols-2">
          <Field label="Name">
            <TextInput value={label} onChange={setLabel} disabled={!card.device} />
          </Field>
          <Field label="Serial port (optional override)" hint="Only for adapters without a serial number. Prefer /dev/serial/by-id/… paths; the port is never hard-coded.">
            <TextInput value={port} onChange={setPort} placeholder="(follow the adapter's identity)" disabled={!card.device} data-port-override />
          </Field>
        </div>
        <div>
          <div className="label mb-1">Calibration profiles</div>
          {profs.length ? (
            <div className="space-y-1.5" data-profiles>
              {profs.map((p) => (
                <div key={p.id} className="flex items-center gap-2 rounded-xl bg-paper px-2.5 py-1.5 text-[12.5px]">
                  <span className="font-medium truncate flex-1">
                    {p.name}
                    {card.calibration?.profile?.id === p.id && <span className="chip-ok text-[10px] ml-1.5">on the motors</span>}
                  </span>
                  <span className="text-[11px] text-ink-3 shrink-0">{new Date(p.created_at).toLocaleString()}</span>
                  <button className="btn-ghost btn-sm" onClick={() => api.calStart(robotType, role, 'apply', p.id).then(() => { onClose(); refreshHardware(); }).catch((e) => toast.error(e.message))} disabled={!card.can_calibrate} title={card.can_calibrate ? 'Write this profile to the motors (torque off, no motion)' : card.calibrate_reason}>
                    Apply
                  </button>
                  <button className="btn-ghost btn-sm text-signal-err" onClick={() => api.profileDelete(p.id).then(load).catch((e) => toast.error(e.message))}>Delete</button>
                </div>
              ))}
            </div>
          ) : (
            <p className="text-[12.5px] text-ink-3">None yet. Calibrate this arm to save one.</p>
          )}
          {profs.length > 0 && (
            <div className="mt-3 flex items-end gap-2">
              <Field label="Copy newest profile to LeRobot's calibration folder" hint="For lerobot-* commands or a LeRobot-based bridge.py. An existing file is kept as a .bak copy." className="flex-1">
                <TextInput value={lid} onChange={setLid} placeholder="LeRobot id, e.g. my_follower" />
              </Field>
              <button
                className="btn btn-sm mb-[22px]"
                disabled={!lid}
                onClick={() => api.profileInstall(profs[0].id, lid).then((r) => toast.success(`Written to ${r.path}`)).catch((e) => toast.error(e.message))}
              >
                Copy
              </button>
            </div>
          )}
        </div>
      </div>
    </Modal>
  );
}

// ------------------------------------------------------------------ calibration ---
const CAL_STEPS = [
  ['ready', 'Prepare'],
  ['torque_off', 'Middle position'],
  ['recording', 'Range of motion'],
  ['done', 'Saved'],
];

function CalibrationModal({ session, card, onClose }) {
  const [name, setName] = useState('');
  const hb = useRef(null);
  const id = session?.id;
  useEffect(() => {
    if (!id || !session.running) return undefined;
    hb.current = setInterval(() => api.calCommand(id, 'heartbeat').catch(() => {}), 2000);
    return () => clearInterval(hb.current);
  }, [id, session?.running]);
  if (!session) return null;
  const ph = session.phase;
  const cmd = (c) => api.calCommand(id, c, name || undefined).then(refreshHardware).catch((e) => toast.error(e.message));
  const done = ['done', 'cancelled', 'failed'].includes(ph);
  const apply = session.mode === 'apply';
  const stepIdx = CAL_STEPS.findIndex(([k]) => k === ph);
  return (
    <Modal
      open
      title={`${apply ? 'Apply calibration' : 'Calibrate'} · ${ROLE_NAME[session.role]}`}
      onClose={() => (done ? api.calDismiss().finally(() => { refreshHardware(); onClose(); }) : toast('Cancel the calibration first.', { icon: '⚠' }))}
      width="max-w-2xl"
      footer={
        done ? (
          <button className="btn-primary" onClick={() => api.calDismiss().finally(() => { refreshHardware(); onClose(); })} data-action="cal-close">Close</button>
        ) : (
          <>
            <button className="btn mr-auto" onClick={() => cmd('cancel')} data-action="cal-cancel">Cancel and restore</button>
            {ph === 'ready' && <button className="btn-primary" onClick={() => cmd('begin')} data-action="cal-begin">{apply ? 'Turn torque off and write profile' : 'Turn torque off'}</button>}
            {ph === 'torque_off' && <button className="btn-primary" onClick={() => cmd('set_middle')} data-action="cal-middle">Arm is in the middle - set homing</button>}
            {ph === 'recording' && <button className="btn-primary" onClick={() => cmd('finish')} data-action="cal-finish">Finish and save</button>}
          </>
        )
      }
    >
      <div className="space-y-4" data-cal-phase={ph}>
        {!apply && (
          <ol className="grid grid-cols-4 gap-1.5">
            {CAL_STEPS.map(([k, l], i) => (
              <li key={k} className={clsx('rounded-xl px-2 py-1.5 text-[11.5px]', i < stepIdx || ph === 'done' ? 'bg-signal-okbg' : i === stepIdx ? 'bg-ink text-white' : 'bg-paper text-ink-3')}>
                <span className="font-mono mr-1">{i + 1}</span>
                {l}
              </li>
            ))}
          </ol>
        )}
        {(ph === 'starting' || ph === 'connecting') && <div className="flex items-center gap-2 text-[12.5px]"><Spinner /> Opening {session.port}…</div>}
        {ph === 'ready' && (
          <Callout tone="warn" title="Support the arm before you continue">
            The next step turns the motor torque OFF on {session.port}: the {session.role} can drop under its own weight. No motion is ever
            commanded. {apply ? 'The profile values are then written to the motors and read back.' : 'You then move the arm by hand.'} Cancel at any
            time writes the original values back.
          </Callout>
        )}
        {ph === 'torque_off' && (
          <Callout tone="info" title="Move every joint to the middle of its range">
            Hold the arm in the middle of its range of motion (like the LeRobot calibration pose), then set the homing. This writes each
            motor's homing offset.
          </Callout>
        )}
        {ph === 'recording' && (
          <>
            <Callout tone="info" title="Move each joint through its whole range">
              Move every joint (except wrist_roll, which turns fully) slowly from one end to the other. The recorded limits are written to
              the motors when you finish.
            </Callout>
            <div className="grid grid-cols-2 gap-1.5" data-cal-ranges>
              {(session.names || []).map((n) => {
                const lo = session.mins?.[n];
                const hi = session.maxes?.[n];
                const p = session.positions?.[n];
                return (
                  <div key={n} className="rounded-xl bg-paper px-2 py-1.5 text-[11.5px]">
                    <div className="flex justify-between"><span className="font-mono">{n}</span><span className="font-mono tabular-nums">{p ?? '—'}</span></div>
                    <div className="h-1.5 mt-1 bg-paper-3 rounded-pill relative">
                      {lo !== undefined && <div className="absolute top-0 h-1.5 rounded-pill bg-sky-ink/40" style={{ left: `${(lo / 4095) * 100}%`, width: `${Math.max(0.5, ((hi - lo) / 4095) * 100)}%` }} />}
                      {p !== undefined && <div className="absolute -top-[1px] h-2 w-1 rounded-pill bg-ink" style={{ left: `${(p / 4095) * 100}%` }} />}
                    </div>
                    <div className="flex justify-between text-ink-3 font-mono mt-0.5"><span>{lo ?? ''}</span><span>{hi ?? ''}</span></div>
                  </div>
                );
              })}
            </div>
            <Field label="Profile name (optional)">
              <TextInput value={name} onChange={setName} placeholder={`${card?.device?.label || session.role} ${new Date().toLocaleDateString()}`} />
            </Field>
          </>
        )}
        {session.warning && <Callout tone="warn">{session.warning}</Callout>}
        {ph === 'done' && (
          <Callout tone="ok" title={apply ? 'Profile written to the motors' : 'Calibration saved'}>
            The motors keep the new values (read back and checked) and a calibration profile was saved. Motor torque stays off; bridge.py
            turns it on again when it starts.
          </Callout>
        )}
        {ph === 'cancelled' && (
          <Callout tone="warn" title="Calibration cancelled">
            {session.end_reason} {session.written ? (session.restored ? 'The original values were written back.' : session.restore_error || '') : 'Nothing was written.'}
          </Callout>
        )}
        {ph === 'failed' && <Callout tone="err" title="Calibration failed">{session.error}</Callout>}
      </div>
    </Modal>
  );
}

// ------------------------------------------------------------------ jog panel ---
const JOG_CHECK = [
  ['area_clear', "The follower's workspace is clear of people and objects."],
  ['estop_ready', 'The arm power switch (hardware emergency stop) is within reach.'],
  ['watching', 'I will watch the arm the whole time.'],
];

// input: 'gui' (hold the ± buttons, the original), 'keyboard' or 'gamepad' (the held key or
// stick picks the joint and direction; still one joint at a time, only while held)
export function JogPanel({ robotType, status, cap, input = 'gui' }) {
  const { call } = usePlanningService();
  const [names, setNames] = useState([]);
  const [limits, setLimits] = useState([]);
  const [datasets, setDatasets] = useState([]);
  const [ds, setDs] = useState('');
  const [checks, setChecks] = useState({});
  const [prep, setPrep] = useState(null);
  const [busy, setBusy] = useState(false);
  const session = status.sessions?.jog;
  const live = session && session.running;
  const held = useRef(null);
  const sessionId = session?.id;

  useEffect(() => {
    call('robot.profile', {})
      .then((res) => {
        const jo = res.params?.joint_order || {};
        const order = jo.leader || Object.values(jo)[0] || [];
        setNames(order);
      })
      .catch(() => {});
    trainerFetch('/datasets').then((r) => setDatasets(r.datasets || [])).catch(() => {});
  }, [call]);

  // keep the session alive while the page is open; a closed page stops the jog
  useEffect(() => {
    if (!live) return undefined;
    const iv = setInterval(() => {
      const body = held.current ? { command: 'move', joint: held.current.joint, dir: held.current.dir } : { command: 'heartbeat' };
      api.jogCommand(sessionId, body).catch(() => {});
    }, 100);
    return () => clearInterval(iv);
  }, [live, sessionId]);

  const fromDataset = async (id) => {
    setDs(id);
    setPrep(null);
    if (!id) return;
    setBusy(true);
    try {
      const r = await api.jogLimits(id);
      const by = Object.fromEntries(r.joints.map((j) => [j.name, j]));
      setLimits(names.map((n) => by[n] || { name: n, min: '', max: '', speed: '' }));
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  const setL = (i, k, v) => {
    setPrep(null);
    setLimits((l) => l.map((j, n) => (n === i ? { ...j, [k]: v } : j)));
  };
  const review = async () => {
    try {
      setPrep(await api.jogPrepare(robotType, { names, limits }));
    } catch (e) {
      toast.error(e.message);
    }
  };
  const start = async () => {
    try {
      await api.jogStart(robotType, { names, limits }, prep.token, checks);
      setChecks({});
      setPrep(null);
      refreshHardware();
    } catch (e) {
      toast.error(e.message);
    }
  };
  const press = (joint, dir) => {
    held.current = { joint, dir };
    api.jogCommand(sessionId, { command: 'move', joint, dir }).catch(() => {});
  };
  const release = () => {
    if (!held.current) return;
    held.current = null;
    api.jogCommand(sessionId, { command: 'release' }).catch(() => {});
  };
  const { pad } = useHeldInput({
    names: session?.names || names,
    keyboard: !!live && input === 'keyboard',
    gamepad: !!live && input === 'gamepad',
    onTick: (v) => {
      const k = Object.keys(v).sort((x, y) => Math.abs(v[y]) - Math.abs(v[x]))[0];
      if (!k) release();
      else if (!held.current || held.current.joint !== k || held.current.dir !== Math.sign(v[k])) press(k, Math.sign(v[k]));
    },
  });

  if (!cap.available && !live) {
    return (
      <Sheet title="Direct robot control (follower jog)" code="J">
        <p className="text-[12.5px] text-ink-3" data-jog-unavailable>{cap.reason}</p>
      </Sheet>
    );
  }
  return (
    <Sheet title="Direct robot control (follower jog)" code="J" soft={live}>
      {!live ? (
        <div className="space-y-3" data-jog-setup>
          <Callout tone="warn" title="This moves the real follower">
            One joint moves at a time, only while you hold its button, at the speed you review here, inside the limits below. Commands go
            through bridge.py on {status.topics?.leader}. Releasing the button, a closed page, stale joint states or another node commanding
            the arm hold it in place.
          </Callout>
          <div className="grid gap-3 md:grid-cols-[minmax(0,320px)_1fr] items-end">
            <Field label="Limits from a recorded dataset" hint="The joint envelope of your demonstrations (as for policy rollouts).">
              <Select value={ds} onChange={fromDataset} options={[{ value: '', label: 'Choose a dataset…' }, ...datasets.map((d) => ({ value: d.id, label: d.id }))]} />
            </Field>
            {busy && <Spinner />}
          </div>
          {limits.length > 0 && (
            <div className="overflow-x-auto">
              <table className="w-full text-[12.5px]" data-jog-limits>
                <thead>
                  <tr className="text-left label">
                    <th className="py-1 pr-2">Joint</th>
                    <th className="pr-2">Min</th>
                    <th className="pr-2">Max</th>
                    <th>Speed / s</th>
                  </tr>
                </thead>
                <tbody>
                  {limits.map((j, i) => (
                    <tr key={j.name} className="border-t border-rule">
                      <td className="py-1 pr-2 font-mono">{j.name}</td>
                      {['min', 'max', 'speed'].map((k) => (
                        <td key={k} className="pr-2 w-[110px]">
                          <TextInput value={String(j[k])} onChange={(v) => setL(i, k, v)} className="py-1" />
                        </td>
                      ))}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          {limits.length > 0 && (
            <button className="btn btn-sm" onClick={review} data-action="jog-review">Review</button>
          )}
          {prep && !prep.ok && <Callout tone="err">{prep.errors.join(' ')}</Callout>}
          {prep && prep.ok && (
            <div className="space-y-2">
              {JOG_CHECK.map(([k, t]) => (
                <label key={k} className="flex gap-2 text-[12.5px]">
                  <input type="checkbox" checked={!!checks[k]} onChange={(e) => setChecks({ ...checks, [k]: e.target.checked })} data-jog-check={k} />
                  {t}
                </label>
              ))}
              <button className="btn-danger" disabled={!JOG_CHECK.every(([k]) => checks[k])} onClick={start} data-action="jog-start">
                Start direct control
              </button>
            </div>
          )}
        </div>
      ) : (
        <div className="space-y-3" data-jog-live={session.phase}>
          {input !== 'gui' && (
            <div className="text-[12px]" data-jog-input={input}>
              {input === 'gamepad' ? (
                <StatusDot tone={pad ? 'ok' : 'idle'} label={pad ? pad.slice(0, 60) : 'No gamepad - connect one and press any button'} />
              ) : (
                <span className="text-ink-3">Hold Q/A, W/S, E/D, R/F, T/G, Y/H for joints 1-6 (+/−).</span>
              )}
            </div>
          )}
          <div className="flex items-center gap-3 flex-wrap">
            <StatusDot tone={session.phase === 'moving' ? 'warn' : session.phase === 'stale' ? 'err' : 'ok'} pulse={session.phase === 'moving'} label={session.phase === 'moving' ? `Moving ${session.moving}` : session.phase === 'stale' ? 'Holding - no fresh joint states' : 'Holding position'} />
            <span className="text-[11.5px] text-ink-3">{session.commands_sent || 0} commands sent</span>
            <button className="btn btn-sm ml-auto" onClick={() => api.jogCommand(session.id, { command: 'stop' }).then(refreshHardware)} data-action="jog-stop">End session</button>
          </div>
          <div className="grid gap-1.5 md:grid-cols-2">
            {(session.names || names).map((n) => (
              <div key={n} className="flex items-center gap-2 rounded-xl bg-paper px-2 py-1.5" data-jog-joint={n}>
                <button
                  className="btn btn-sm w-9 justify-center select-none"
                  onPointerDown={() => press(n, -1)}
                  onPointerUp={release}
                  onPointerLeave={release}
                  aria-label={`${n} minus`}
                >
                  −
                </button>
                <div className="flex-1 min-w-0">
                  <div className="font-mono text-[12px] truncate">{n}</div>
                  <div className="font-mono text-[11px] text-ink-3 tabular-nums">
                    {fmt(session.measured?.[n])}
                    {session.target?.[n] !== undefined && session.moving === n ? ` → ${fmt(session.target[n])}` : ''}
                    {session.at_limit === n ? ' · at limit' : ''}
                  </div>
                </div>
                <button
                  className="btn btn-sm w-9 justify-center select-none"
                  onPointerDown={() => press(n, 1)}
                  onPointerUp={release}
                  onPointerLeave={release}
                  aria-label={`${n} plus`}
                >
                  +
                </button>
              </div>
            ))}
          </div>
        </div>
      )}
      {session && !session.running && (session.error || session.end_reason) && (
        <Callout tone={session.error ? 'err' : 'info'} className="mt-3">{session.error || session.end_reason}</Callout>
      )}
    </Sheet>
  );
}

// ---------------------------------------------------------------- leader capture ---
function LeaderCapture({ robotType, status, cap, rosbridgeUrl }) {
  const card = status.arms.leader;
  const [on, setOn] = useState(false);
  const [n, setN] = useState(0);
  const [list, setList] = useState([]);
  const buf = useRef({ names: [], samples: [], t0: 0 });
  const load = useCallback(() => api.captures(robotType).then((r) => setList(r.captures || [])).catch(() => {}), [robotType]);
  useEffect(() => {
    load();
  }, [load]);
  useEffect(() => {
    if (!on) return undefined;
    buf.current = { names: [], samples: [], t0: performance.now() };
    let stop = () => {};
    let alive = true;
    const push = (names, values) => {
      if (!buf.current.names.length) buf.current.names = names;
      if (names.length !== buf.current.names.length) return;
      buf.current.samples.push([(performance.now() - buf.current.t0) / 1000, values.map(Number)]);
      setN(buf.current.samples.length);
    };
    if (card.source === 'ros' && card.topic) {
      rosConnectionManager.getConnection(rosbridgeUrl).then((ros) => {
        if (!alive) return;
        const t = new ROSLIB.Topic({ ros, name: card.topic, messageType: 'trajectory_msgs/msg/JointTrajectory', throttle_rate: 33 });
        const cb = (m) => push(m.joint_names || [], (m.points && m.points[0] && m.points[0].positions) || []);
        t.subscribe(cb);
        stop = () => t.unsubscribe(cb);
      });
    } else {
      const iv = setInterval(async () => {
        try {
          const st = await api.hwStatus(robotType);
          const js = st.arms.leader.joints || {};
          push(Object.keys(js), Object.values(js).map((j) => j.value));
        } catch (e) {
          /* keep going */
        }
      }, 100);
      stop = () => clearInterval(iv);
    }
    return () => {
      alive = false;
      stop();
    };
  }, [on, card.source, card.topic, robotType, rosbridgeUrl]);
  const finish = async () => {
    setOn(false);
    try {
      const r = await api.saveCapture(robotType, { role: 'leader', names: buf.current.names, samples: buf.current.samples, source: card.source });
      toast.success(`Saved ${r.count} samples (${r.duration_s.toFixed(1)} s)`);
      load();
    } catch (e) {
      toast.error(e.message);
    }
  };
  return (
    <Sheet title="Leader data capture" code="LC">
      {!cap.available && !on ? (
        <p className="text-[12.5px] text-ink-3">{cap.reason}</p>
      ) : (
        <div className="space-y-3" data-leader-capture>
          <p className="text-[12.5px] text-ink-3">
            Records the leader's joint trajectory ({card.source === 'ros' ? `from ${card.topic}` : 'motor ticks, read-only session'}) to a file in
            RawBot Studio's data folder. It is not a LeRobot dataset: recording demonstrations needs the follower and cameras.
          </p>
          <div className="flex items-center gap-2">
            {!on ? (
              <button className="btn-primary btn-sm" onClick={() => setOn(true)} data-action="capture-start">Start capture</button>
            ) : (
              <button className="btn-danger btn-sm" onClick={finish} data-action="capture-stop">Stop and save</button>
            )}
            {on && <span className="text-[12px] font-mono">{n} samples</span>}
          </div>
        </div>
      )}
      {list.length > 0 && (
        <div className="mt-3 text-[12px] space-y-0.5">
          <div className="label">Saved captures</div>
          {list.slice(0, 5).map((c) => (
            <div key={c.id} className="flex gap-2"><span className="font-mono">{c.id}</span><span className="text-ink-3">{c.count} samples · {c.duration_s} s · {c.names.length} joints</span></div>
          ))}
        </div>
      )}
    </Sheet>
  );
}

// ------------------------------------------------------------------------ page ---
export default function HardwareSetup() {
  const dispatch = useDispatch();
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const status = useSelector((s) => (s.hardware.status?.robot_type === robotType ? s.hardware.status : null));
  const offline = useSelector((s) => s.hardware.offline);
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const onStatus = useCallback((st) => dispatch(setHardwareStatus(st)), [dispatch]);
  const [busy, run] = useAction(robotType, onStatus);
  const [cfgRole, setCfgRole] = useState(null);
  const [calOpen, setCalOpen] = useState(false);
  const cal = status?.sessions?.calibration;

  useEffect(() => {
    if (cal && !calOpen) setCalOpen(true);
  }, [cal, calOpen]);

  if (!robotType) {
    return (
      <Sheet title="Hardware setup">
        <Callout tone="info">Select and confirm a robot on Home first. Its hardware is set up here.</Callout>
      </Sheet>
    );
  }
  if (!status) {
    return (
      <Sheet title="Hardware setup">
        {offline ? (
          <Callout tone="err" title="The host service is not reachable">
            Hardware detection runs in the RawBot Studio training service on this PC. Start it with ./start_physios.sh. Until then pages
            work as before, without hardware checks.
          </Callout>
        ) : (
          <div className="py-6 flex justify-center"><Spinner /></div>
        )}
      </Sheet>
    );
  }
  const used = (MODE_OPTIONS.find((m) => m.value === status.mode) || {}).value;
  const scan = status.scan_supported;
  return (
    <div className="grid gap-4" data-hardware-setup data-mode={status.mode}>
      <Sheet
        title={`Hardware · ${robotType}`}
        code="HW"
        actions={
          <>
            <button className="btn btn-sm" onClick={() => run('refresh', () => api.hwStatus(robotType))} disabled={!!busy} data-action="hw-refresh">
              {busy === 'refresh' ? <Spinner /> : '↻'} Refresh Status
            </button>
            <button className="btn-primary btn-sm" onClick={() => run('detect', () => api.hwDetect(robotType))} disabled={!!busy || status.mode === 'simulation'} data-action="hw-detect">
              {busy === 'detect' && <Spinner className="border-paper border-t-transparent" />} Detect Hardware
            </button>
            <button className="btn-danger btn-sm" onClick={() => api.hwEstop('page').then((r) => toast(`Emergency stop: ${r.stopped.join(', ') || 'nothing was moving'}`, { icon: '■' }))} data-action="hw-estop" title="Stops jog, rollout and calibration. Also available without a browser: ./scripts/estop.sh">
              ■ Stop all
            </button>
          </>
        }
      >
        <div className="grid gap-3 lg:grid-cols-[auto_1fr] items-center">
          <div>
            <div className="label mb-1">Hardware configuration</div>
            <Segmented
              value={used}
              onChange={(v) => run('mode', () => api.hwSetup(robotType, { mode: v }))}
              options={MODE_OPTIONS.map((m) => ({ value: m.value, label: m.label }))}
              data-mode-select
            />
          </div>
          <div className="text-[12.5px] text-ink-3 lg:pl-4">
            {status.mode === 'simulation'
              ? 'No physical robot: planning, the 3D model, the simulation view and training work; recording and anything that moves an arm are off.'
              : status.mode === 'leader_sim'
              ? 'One physical arm: the leader drives the simulated follower in real time. Connect the leader here (read-only session, no bridge.py needed) or through bridge.py; record demonstrations from Planning → Robot & control.'
              : status.mode === 'leader_only'
              ? 'Only the leader arm: calibrate it, monitor its joints and capture its trajectories. Connecting a follower later enables Leader + Follower.'
              : status.mode === 'follower_only'
              ? 'Only the follower arm: calibrate it, control it directly, run policies and record hand-guided demonstrations. Connecting a leader later enables Leader + Follower.'
              : 'Both arms: teleoperation and synchronized demonstration recording (the original workflow).'}
            {status.mode_source === 'auto' && <span className="chip-sky text-[10px] ml-1.5">switched automatically</span>}
          </div>
        </div>
        <div className="mt-3 flex flex-wrap gap-x-5 gap-y-1 text-[11.5px] text-ink-3">
          <span>
            <StatusDot tone={status.ros.connected ? 'ok' : 'err'} label={status.ros.connected ? 'ROS (rosbridge) reachable' : 'ROS not reachable'} />
          </span>
          <span>Follower topic <span className="font-mono">{status.topics?.follower || '—'}</span></span>
          <span>Leader / command topic <span className="font-mono">{status.topics?.leader || '—'}</span></span>
          {!scan && <span>Motor scans: SO-101 only - this robot is detected through ROS.</span>}
          {!status.platform_ports && <span>Port list from the LeRobot environment (no Linux sysfs).</span>}
        </div>
      </Sheet>

      <div className="grid gap-4 lg:grid-cols-2">
        {['leader', 'follower'].map((role) => (
          <ArmCard
            key={role}
            robotType={robotType}
            card={status.arms[role]}
            busy={busy}
            run={run}
            rosbridgeUrl={rosbridgeUrl}
            onConfigure={setCfgRole}
            onCalibrate={(r) =>
              run(`cal-${r}`, async () => {
                await api.calStart(robotType, r);
                setCalOpen(true);
                return api.hwStatus(robotType);
              })
            }
          />
        ))}
      </div>

      {status.mode !== 'simulation' && <Unassigned robotType={robotType} status={status} busy={busy} run={run} />}
      <Capabilities status={status} />
      {['leader_only', 'leader_sim', 'leader_follower'].includes(status.mode) && status.arms.leader.state !== 'unused' && (
        <LeaderCapture robotType={robotType} status={status} cap={status.capabilities.leader_capture} rosbridgeUrl={rosbridgeUrl} />
      )}
      {['follower_only', 'leader_follower'].includes(status.mode) && (
        <JogPanel robotType={robotType} status={status} cap={status.capabilities.follower_control} />
      )}
      {cfgRole && <ConfigureModal robotType={robotType} role={cfgRole} card={status.arms[cfgRole]} onClose={() => setCfgRole(null)} onStatus={onStatus} />}
      {calOpen && cal && <CalibrationModal session={cal} card={status.arms[cal.role]} onClose={() => setCalOpen(false)} />}
    </div>
  );
}
