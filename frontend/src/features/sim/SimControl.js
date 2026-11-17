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

// Control of the simulated robot (host service, hardware/simbot.py) for the chosen action
// source: GUI sliders, keyboard, gamepad, a waypoint script or a recorded trajectory. The
// simulated robot never commands a physical arm.

import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import toast from 'react-hot-toast';
import clsx from 'clsx';
import { NumberInput, Select, Spinner, StatusDot, Toggle } from '../../components/ui';
import rosConnectionManager from '../../utils/rosConnectionManager';
import { usePlanningService } from '../../hooks/usePlanningService';
import { trainerFetch } from '../../hooks/useTrainer';
import { captures as listCaptures, hwPlanning, hwSim, jogLimits } from '../hardware/hardwareApi';
import { setHardwareStatus } from '../hardware/hardwareSlice';
import useHeldInput, { KEY_PAIRS, PAD_MAP } from './useHeldInput';

const fmt = (v) => (typeof v === 'number' ? (Math.abs(v) < 0.0005 ? 0 : v).toFixed(3) : '—');

// live joint states of the simulated robot (30 Hz over rosbridge); the 2 s hardware status
// is the fallback when rosbridge does not deliver them
export function useSimJoints(sim) {
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const [msg, setMsg] = useState(null);
  const running = !!sim?.running;
  const topic = sim?.topic;
  useEffect(() => {
    if (!running || !topic) return undefined;
    let sub = null;
    let alive = true;
    let lastT = 0;
    rosConnectionManager
      .getConnection(rosbridgeUrl)
      .then((ros) => {
        if (!alive) return;
        sub = new ROSLIB.Topic({ ros, name: topic, messageType: 'sensor_msgs/msg/JointState', throttle_rate: 66 });
        sub.subscribe((m) => {
          const t = Date.now();
          if (t - lastT < 60) return;
          lastT = t;
          setMsg({ names: m.name, positions: m.position, t });
        });
      })
      .catch(() => {});
    return () => {
      alive = false;
      if (sub) sub.unsubscribe();
    };
  }, [running, topic, rosbridgeUrl]);
  const fresh = msg && Date.now() - msg.t < 1500;
  return fresh ? msg : { names: sim?.names || [], positions: sim?.positions || [] };
}

export default function SimControl({ robotType, sim, action, compact }) {
  const dispatch = useDispatch();
  const live = useSimJoints(sim);
  const names = sim?.names || [];
  const send = useCallback(
    async (body, quiet) => {
      try {
        const st = await hwSim(robotType, body);
        dispatch(setHardwareStatus(st));
        return st;
      } catch (e) {
        if (!quiet) toast.error(e.message);
        return null;
      }
    },
    [robotType, dispatch]
  );

  if (!sim?.running) return null;
  const pos = (n) => {
    const i = live.names.indexOf(n);
    return i >= 0 ? live.positions[i] : undefined;
  };
  return (
    <div className="space-y-3" data-sim-control={action}>
      <div className="flex flex-wrap items-center gap-2 text-[12px]">
        <StatusDot
          tone={sim.error ? 'err' : sim.drive === 'manual' ? 'ok' : 'sky'}
          pulse={sim.drive !== 'manual'}
          label={sim.error ? 'Not publishing' : `${sim.state_topic ? 'Simulated follower' : 'Simulated robot'} · ${DRIVE_LABEL[sim.drive] || sim.drive}`}
        />
        {sim.state_topic && <span className="chip num" title="physical_ai_server records this topic as the robot state">{sim.state_topic}</span>}
        <span className="flex-1" />
        {sim.capture ? (
          <button
            className="btn-danger btn-sm"
            onClick={async () => {
              const st = await send({ command: 'capture_stop' });
              if (st?.saved_capture) toast.success(`Trajectory saved (${st.saved_capture.count} samples, ${st.saved_capture.duration_s.toFixed(1)} s)`);
            }}
            data-action="sim-capture-stop"
          >
            ■ Save trajectory · {sim.capture.duration_s.toFixed(1)} s
          </button>
        ) : (
          <button
            className="btn btn-sm"
            onClick={() => send({ command: 'capture_start' })}
            title="Joint-only trajectory of the simulated robot, saved with the leader captures (no cameras needed)"
            data-action="sim-capture-start"
          >
            ● Capture
          </button>
        )}
        <button className="btn btn-sm" onClick={() => send({ command: 'home' })} data-action="sim-home">
          Home
        </button>
        <button className="btn btn-sm" onClick={() => send({ command: 'stop' })} data-action="sim-stop">
          ■ Stop
        </button>
      </div>
      {sim.error && <div className="text-[12px] text-signal-err">{sim.error}</div>}
      {action === 'gui' && <Sliders names={names} sim={sim} pos={pos} send={send} />}
      {(action === 'keyboard' || action === 'gamepad') && (
        <HeldControl names={names} kind={action} pos={pos} send={send} />
      )}
      {action === 'script' && <Script robotType={robotType} names={names} sim={sim} live={live} send={send} />}
      {action === 'replay' && <Replay robotType={robotType} sim={sim} send={send} />}
      {action === 'leader' && <LeaderSync robotType={robotType} sim={sim} pos={pos} compact={compact} send={send} />}
      {!compact && action !== 'gui' && action !== 'leader' && <JointReadout names={names} pos={pos} />}
    </div>
  );
}

const DRIVE_LABEL = {
  manual: 'manual control',
  leader: 'following the leader arm',
  follower: 'mirroring the physical follower',
  script: 'running the script',
  replay: 'replaying a trajectory',
};

// live leader joints: RawBot Studio's own topic when it reads the leader over serial, else the
// leader topic of bridge.py; the 2 s hardware status is the fallback
function useLeaderJoints(li) {
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const [msg, setMsg] = useState(null);
  const topic = li?.published || li?.topic;
  const type = li?.published ? 'sensor_msgs/msg/JointState' : 'trajectory_msgs/msg/JointTrajectory';
  useEffect(() => {
    if (!topic) return undefined;
    let sub = null;
    let alive = true;
    let lastT = 0;
    rosConnectionManager
      .getConnection(rosbridgeUrl)
      .then((ros) => {
        if (!alive) return;
        sub = new ROSLIB.Topic({ ros, name: topic, messageType: type, throttle_rate: 66 });
        sub.subscribe((m) => {
          const t = Date.now();
          if (t - lastT < 60) return;
          lastT = t;
          const names = m.name || m.joint_names || [];
          const positions = m.position || (m.points && m.points[0] && m.points[0].positions) || [];
          setMsg({ names, positions, t });
        });
      })
      .catch(() => {});
    return () => {
      alive = false;
      if (sub) sub.unsubscribe();
    };
  }, [topic, type, rosbridgeUrl]);
  const fresh = msg && Date.now() - msg.t < 1500;
  return fresh ? msg : { names: li?.names || [], positions: li?.values || [] };
}

// Physical leader -> simulated follower: joint mapping by name, the conversion from motor
// ticks (serial session), joint limits, and what the simulated follower actually does
function LeaderSync({ robotType, sim, pos, compact, send }) {
  const dispatch = useDispatch();
  const li = sim.leader_in || {};
  const lead = useLeaderJoints(li);
  const names = sim.names || [];
  const serial = li.source === 'serial';
  const [ds, setDs] = useState('');
  const [datasets, setDatasets] = useState([]);
  const [map, setMap] = useState(null);
  const jm = map || sim.joint_map || {};
  useEffect(() => {
    trainerFetch('/datasets')
      .then((r) => setDatasets(r.datasets || []))
      .catch(() => {});
  }, []);
  const lv = (n) => {
    const i = lead.names.indexOf((li.pairs || {})[n] || n);
    return i >= 0 ? Number(lead.positions[i]) : undefined;
  };
  const ticks = (n) => {
    const i = (li.names || []).indexOf(n);
    return i >= 0 && li.ticks ? li.ticks[i] : undefined;
  };
  const limitsFrom = async (id) => {
    setDs(id);
    try {
      if (!id) {
        await send({ command: 'limits', limits: Object.fromEntries(names.map((n) => [n, [-Math.PI, Math.PI]])) });
        return;
      }
      const r = await jogLimits(id);
      const lim = {};
      r.joints.forEach((j) => {
        if (names.includes(j.name) && j.min !== '' && j.max !== '' && Number(j.min) < Number(j.max)) lim[j.name] = [Number(j.min), Number(j.max)];
      });
      await send({ command: 'limits', limits: lim });
      toast.success(`Joint limits from ${id}`);
    } catch (e) {
      toast.error(e.message);
    }
  };
  const saveMap = async () => {
    try {
      dispatch(setHardwareStatus(await hwPlanning(robotType, { joint_map: jm })));
      setMap(null);
      toast.success('Joint mapping saved');
    } catch (e) {
      toast.error(e.message);
    }
  };
  const setM = (n, k, v) => setMap({ ...jm, [n]: { sign: 1, offset: 0, ...(jm[n] || {}), [k]: v } });
  return (
    <div className="space-y-2" data-leader-sync={li.stale ? 'stale' : 'live'}>
      <div className="flex flex-wrap items-center gap-x-4 gap-y-1 text-[12px]">
        <StatusDot
          tone={li.stale ? 'warn' : 'ok'}
          pulse={!li.stale}
          label={
            li.stale
              ? 'No fresh leader readings - the simulated follower holds'
              : serial
              ? `Leader · serial session${li.rate_hz ? ` ${li.rate_hz} Hz` : ''}`
              : `Leader · bridge.py (${li.topic || 'leader topic'})`
          }
        />
        {li.published && <span className="chip num" title="Recorded as the action (action_source leader_state)">{li.published}</span>}
        {(li.at_limit || []).length > 0 && <span className="chip-warn">at limit: {li.at_limit.join(', ')}</span>}
        {(li.missing || []).length > 0 && <span className="chip-warn">no leader joint: {li.missing.join(', ')}</span>}
      </div>
      <table className="w-full text-[11.5px] font-mono" data-leader-sync-table>
        <thead>
          <tr className="text-left text-ink-4">
            <th className="font-normal">joint</th>
            {serial && <th className="font-normal text-right">ticks</th>}
            <th className="font-normal text-right">leader</th>
            <th className="font-normal text-right">sim</th>
            <th className="font-normal text-right">Δ</th>
          </tr>
        </thead>
        <tbody>
          {names.map((n, i) => {
            const a = lv(n);
            const b = pos(n);
            const d = a !== undefined && b !== undefined ? b - a : undefined;
            const lim = (li.at_limit || []).includes(n);
            return (
              <tr key={n} className="border-t border-rule" data-sync-joint={n}>
                <td className="truncate max-w-[120px]">{n}</td>
                {serial && <td className="text-right tabular-nums text-ink-3">{ticks(n) ?? '—'}</td>}
                <td className="text-right tabular-nums">{fmt(a)}</td>
                <td className={clsx('text-right tabular-nums', lim && 'text-signal-warn')}>{fmt(b)}</td>
                <td className={clsx('text-right tabular-nums', d !== undefined && Math.abs(d) > 0.1 && 'text-signal-warn')}>{fmt(d)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <JointPairs robotType={robotType} li={li} dispatch={dispatch} />
      {!compact && (
        <div className="grid gap-2 sm:grid-cols-[auto_1fr] sm:items-center text-[12px]">
          <span className="label">Joint limits</span>
          <Select
            value={ds}
            onChange={limitsFrom}
            options={[{ value: '', label: '±π (no dataset)' }, ...datasets.map((d) => ({ value: d.id, label: `From ${d.id}` }))]}
          />
        </div>
      )}
      {!compact && serial && (
        <details className="text-[12px]" data-joint-map>
          <summary className="cursor-pointer text-ink-3 hover:text-ink">Joint mapping (ticks → radians)</summary>
          <p className="mt-1 text-[11.5px] text-ink-4">(ticks − 2048) × 2π / 4096, then sign and offset. Compare with a recorded dataset before recording.</p>
          <div className="mt-1.5 grid gap-1">
            {names.map((n) => (
              <div key={n} className="grid grid-cols-[110px_60px_1fr] items-center gap-2" data-map-joint={n}>
                <span className="font-mono truncate">{n}</span>
                <button type="button" className="btn btn-sm justify-center" onClick={() => setM(n, 'sign', (jm[n]?.sign || 1) === 1 ? -1 : 1)} aria-label={`${n} direction`}>
                  {(jm[n]?.sign || 1) === 1 ? '+1' : '−1'}
                </button>
                <NumberInput value={jm[n]?.offset || 0} step={0.01} onChange={(v) => setM(n, 'offset', Number(v) || 0)} suffix="rad" className="py-1" />
              </div>
            ))}
          </div>
          <button className="btn btn-sm mt-2" disabled={!map} onClick={saveMap} data-action="save-joint-map">
            Save mapping
          </button>
        </details>
      )}
    </div>
  );
}

// mixed robots: a target joint with no same-named leader joint takes one picked here. Shown
// only when something is unmatched or paired.
function JointPairs({ robotType, li, dispatch }) {
  const pairs = li.pairs || {};
  const rows = [...new Set([...(li.missing || []), ...Object.keys(pairs)])];
  if (!rows.length || !(li.names || []).length) return null;
  const save = async (target, source) => {
    const next = { ...pairs };
    if (source) next[target] = source;
    else delete next[target];
    try {
      dispatch(setHardwareStatus(await hwPlanning(robotType, { joint_pairs: next })));
    } catch (e) {
      toast.error(e.message);
    }
  };
  return (
    <div className="grid gap-1 text-[12px]" data-joint-pairs>
      {rows.map((t) => (
        <label key={t} className="grid grid-cols-[110px_auto_1fr] items-center gap-2" data-pair={t}>
          <span className={clsx('font-mono truncate', !pairs[t] && 'text-signal-warn')}>{t}</span>
          <span className="text-ink-4">←</span>
          <Select
            value={pairs[t] || ''}
            onChange={(v) => save(t, v)}
            options={[{ value: '', label: pairs[t] ? 'same name' : 'pick a leader joint' }, ...li.names.map((n) => ({ value: n, label: n }))]}
          />
        </label>
      ))}
    </div>
  );
}

function JointReadout({ names, pos }) {
  return (
    <div className="grid grid-cols-2 gap-x-4 gap-y-0.5 font-mono text-[11.5px]" data-sim-joints>
      {names.map((n) => (
        <div key={n} className="flex justify-between gap-2 min-w-0">
          <span className="truncate text-ink-3">{n}</span>
          <span className="tabular-nums">{fmt(pos(n))}</span>
        </div>
      ))}
    </div>
  );
}

function Sliders({ names, sim, pos, send }) {
  const pending = useRef({});
  const timer = useRef(null);
  const [vals, setVals] = useState({});
  const set = (n, v) => {
    setVals((x) => ({ ...x, [n]: v }));
    pending.current[n] = v;
    if (timer.current) return;
    timer.current = setTimeout(() => {
      const t = pending.current;
      pending.current = {};
      timer.current = null;
      send({ command: 'targets', targets: t }, true);
    }, 70);
  };
  useEffect(() => () => timer.current && clearTimeout(timer.current), []);
  return (
    <div className="space-y-1.5" data-sim-sliders>
      {names.map((n, i) => {
        const [lo, hi] = sim.limits?.[i] || [-3.14, 3.14];
        const v = vals[n] ?? sim.target?.[i] ?? 0;
        return (
          <label key={n} className="grid grid-cols-[110px_1fr_64px] items-center gap-2 text-[12px]">
            <span className="font-mono truncate">{n}</span>
            <input
              type="range"
              min={lo}
              max={hi}
              step={0.01}
              value={v}
              onChange={(e) => set(n, Number(e.target.value))}
              data-sim-slider={n}
              className="w-full accent-sky-ink"
            />
            <span className="font-mono text-[11px] tabular-nums text-right text-ink-3">{fmt(pos(n))}</span>
          </label>
        );
      })}
    </div>
  );
}

function HeldControl({ names, kind, pos, send }) {
  const { held, pad } = useHeldInput({
    names,
    keyboard: kind === 'keyboard',
    gamepad: kind === 'gamepad',
    onTick: (v) => send(Object.keys(v).length ? { command: 'velocity', velocity: v } : { command: 'release' }, true),
  });
  return (
    <div className="space-y-2" data-sim-held={kind}>
      {kind === 'gamepad' && (
        <div className="text-[12px]">
          <StatusDot tone={pad ? 'ok' : 'idle'} label={pad ? pad.slice(0, 60) : 'No gamepad - connect one and press any button'} />
        </div>
      )}
      <div className="grid gap-1 sm:grid-cols-2">
        {names.map((n, i) => (
          <div
            key={n}
            className={clsx('flex items-center gap-2 rounded-lg px-2 py-1 text-[12px]', held[n] ? 'bg-sky-50' : 'bg-paper')}
            data-held-joint={n}
          >
            <span className="font-mono text-[10.5px] text-ink-3 w-[86px] shrink-0">
              {kind === 'keyboard' ? (KEY_PAIRS[i] ? `${KEY_PAIRS[i][0].toUpperCase()} + / ${KEY_PAIRS[i][1].toUpperCase()} −` : '—') : PAD_MAP[i] || '—'}
            </span>
            <span className="font-mono truncate flex-1">{n}</span>
            <span className="font-mono tabular-nums text-[11px] text-ink-3">{fmt(pos(n))}</span>
          </div>
        ))}
      </div>
      <p className="text-[11.5px] text-ink-4">Moves only while held.</p>
    </div>
  );
}

const scriptKey = (rt) => `rawbot.simscript.${rt}`;

function Script({ robotType, names, sim, live, send }) {
  const [points, setPoints] = useState(() => {
    try {
      return JSON.parse(window.localStorage.getItem(scriptKey(robotType)) || '[]');
    } catch (e) {
      return [];
    }
  });
  const [loop, setLoop] = useState(false);
  const [hold, setHold] = useState(0.5);
  const save = (p) => {
    setPoints(p);
    try {
      window.localStorage.setItem(scriptKey(robotType), JSON.stringify(p));
    } catch (e) {
      /* per-browser convenience only */
    }
  };
  const add = () => {
    const row = names.map((n) => {
      const i = live.names.indexOf(n);
      return i >= 0 ? Number(live.positions[i]) : 0;
    });
    save([...points, row]);
  };
  const playing = sim.drive === 'script' && sim.script && !sim.script.done;
  return (
    <div className="space-y-2" data-sim-script>
      <p className="text-[12px] text-ink-3">Pose the simulated robot (Home, or switch to GUI control), add waypoints, then play them in order.</p>
      <div className="flex flex-wrap items-center gap-2">
        <button className="btn btn-sm" onClick={add} data-action="sim-add-waypoint">
          + Add current pose
        </button>
        <button className="btn btn-sm" onClick={() => save([])} disabled={!points.length}>
          Clear
        </button>
        <Toggle checked={loop} onChange={setLoop} label={<span className="text-[12px]">Loop</span>} />
        <span className="flex items-center gap-1 text-[12px]">
          Hold
          <NumberInput value={hold} min={0} step={0.1} onChange={setHold} suffix="s" className="w-[90px] py-1" />
        </span>
        <button
          className="btn-primary btn-sm ml-auto"
          disabled={!points.length}
          onClick={() => send({ command: 'script', points, loop, hold_s: hold })}
          data-action="sim-play-script"
        >
          ▶ Play
        </button>
      </div>
      {points.length > 0 && (
        <ol className="space-y-0.5 text-[11.5px] font-mono" data-sim-waypoints={points.length}>
          {points.map((p, i) => (
            <li key={i} className={clsx('flex gap-2 rounded px-1.5', playing && sim.script.index === i && 'bg-sky-50')}>
              <span className="text-ink-4 w-5">{i + 1}</span>
              <span className="truncate flex-1">{p.map((v) => v.toFixed(2)).join('  ')}</span>
              <button className="text-ink-4 hover:text-signal-err" onClick={() => save(points.filter((_, k) => k !== i))} aria-label={`Remove waypoint ${i + 1}`}>
                ×
              </button>
            </li>
          ))}
        </ol>
      )}
      {sim.script && (
        <div className="text-[11.5px] text-ink-3">
          {sim.script.done ? 'Script finished.' : `Waypoint ${sim.script.index + 1} of ${sim.script.count}${sim.script.loop ? ' · looping' : ''}`}
        </div>
      )}
    </div>
  );
}

function Replay({ robotType, sim, send }) {
  const { call } = usePlanningService();
  const [from, setFrom] = useState('dataset');
  const [caps, setCaps] = useState([]);
  const [cap, setCap] = useState('');
  const [datasets, setDatasets] = useState([]);
  const [ds, setDs] = useState('');
  const [ep, setEp] = useState(0);
  const [speed, setSpeed] = useState(1);
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    trainerFetch('/datasets')
      .then((r) => setDatasets(r.datasets || []))
      .catch(() => {});
    listCaptures(robotType)
      .then((r) => setCaps(r.captures || []))
      .catch(() => {});
  }, [robotType, sim.capture]);
  const play = async () => {
    setBusy(true);
    try {
      if (from === 'capture') {
        await send({ command: 'replay', capture: cap, speed });
        return;
      }
      const ts = await call('data.episode', { dataset_id: ds, episode_index: Number(ep), max_points: 1200 }, { timeoutMs: 30000 });
      const useAction = ts.action && ts.action.length;
      const rows = useAction ? ts.action : ts.state;
      const names = useAction ? ts.action_names : ts.state_names;
      if (!rows || !rows.length) throw new Error('That episode has no joint data.');
      await send({ command: 'replay', names, rows, t: ts.timestamp, speed, label: `${ds} #${ep}` });
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  const r = sim.replay;
  return (
    <div className="space-y-2" data-sim-replay>
      <div className="flex gap-1.5" data-replay-from={from}>
        {[
          ['dataset', 'Dataset episode'],
          ['capture', `Saved capture${caps.length ? ` (${caps.length})` : ''}`],
        ].map(([k, l]) => (
          <button key={k} type="button" className={from === k ? 'chip-ink' : 'chip'} onClick={() => setFrom(k)} aria-pressed={from === k}>
            {l}
          </button>
        ))}
      </div>
      {from === 'capture' ? (
        <div className="grid gap-2 sm:grid-cols-[1fr_90px] items-end">
          <Select
            value={cap}
            onChange={setCap}
            options={[
              { value: '', label: caps.length ? 'Choose a capture…' : 'No saved captures yet' },
              ...caps.map((c) => ({ value: c.id, label: `${c.id} · ${c.role} · ${c.duration_s} s` })),
            ]}
          />
          <NumberInput value={speed} min={0.1} max={4} step={0.25} onChange={setSpeed} suffix="×" className="py-1" />
        </div>
      ) : (
      <div className="grid gap-2 sm:grid-cols-[1fr_90px_90px] items-end">
        <Select
          value={ds}
          onChange={setDs}
          options={[{ value: '', label: 'Choose a recorded dataset…' }, ...datasets.map((d) => ({ value: d.id, label: d.id }))]}
        />
        <NumberInput value={ep} min={0} onChange={setEp} className="py-1" aria-label="Episode" />
        <NumberInput value={speed} min={0.1} max={4} step={0.25} onChange={setSpeed} suffix="×" className="py-1" />
      </div>
      )}
      <button className="btn-primary btn-sm" disabled={(from === 'capture' ? !cap : !ds) || busy} onClick={play} data-action="sim-replay">
        {busy ? <Spinner className="border-paper border-t-transparent" /> : '▶'} {from === 'capture' ? 'Replay capture' : `Replay episode ${ep}`}
      </button>
      {r && (
        <div className="text-[11.5px] text-ink-3" data-sim-replay-state={r.done ? 'done' : 'playing'}>
          {r.label} · {r.elapsed_s.toFixed(1)} / {r.duration_s.toFixed(1)} s{r.done ? ' · finished' : ''}
        </div>
      )}
    </div>
  );
}
