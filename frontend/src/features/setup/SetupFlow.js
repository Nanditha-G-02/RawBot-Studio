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

// Guided robot setup, before Planning:
//   1 Robot     choose an existing robot, or add a custom one
//   2 Model     custom robots: the URDF / Xacro (or a ZIP of the package) comes first; the
//               name is taken from the file, details are optional and editable afterwards
//   3 Hardware  which arms you have (Leader + Follower, Leader Only, Follower Only,
//               Simulation Only), detection and connection
//   4 Verify    robot server, model, hardware (built-in robots); configuration and readiness
//               stages (custom robots)
//   5 Planning  summary, then on to the Planning session
// It reuses the existing pieces (robot server calls, custom robot profiles, hardware setup);
// nothing here moves the robot.

import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import toast from 'react-hot-toast';
import clsx from 'clsx';
import { Callout, Field, Segmented, Select, Sheet, Spinner, TextInput } from '../../components/ui';
import { useRosServiceCaller } from '../../hooks/useRosServiceCaller';
import TaskPhase from '../../constants/taskPhases';
import PageType from '../../constants/pageType';
import { selectRobotType, removeAllTags } from '../tasks/taskSlice';
import { setRobotTypeList, setIsFirstLoadTrue, moveToPage, setRobotsView } from '../ui/uiSlice';
import {
  ROBOT_TYPE_LABELS,
  createRobot,
  fetchResolvedUrdf,
  getRobot,
  listRobots,
  registerRobot,
  updateRobot,
  validateRobot,
} from '../robots/robotsApi';
import { FilePicker, uploadAndValidate } from '../robots/AddRobotWizard';
import { STAGE_NAMES, StageBadge } from '../robots/RobotDetail';
import { robotExtraTabs } from '../robots/RobotSetup';
import UrdfViewer from '../robots/UrdfViewer';
import ValidationReport from '../robots/ValidationReport';
import HardwareSetup from '../hardware/HardwareSetup';
import { MODE_OPTIONS, STATE_LABEL, hwSetup, hwStatus } from '../hardware/hardwareApi';
import ArmChoice from '../hardware/ArmChoice';
import { setHardwareStatus } from '../hardware/hardwareSlice';
import { useHardwareStatus } from '../hardware/useHardwareGate';

export const customKey = (id) => id.toLowerCase().replace(/[^a-z0-9_]/g, '_');

const STEPS = [
  { n: 1, key: 'robot', title: 'Robot', sub: 'Choose or add' },
  { n: 2, key: 'model', title: 'Model file', sub: 'URDF / Xacro' },
  { n: 3, key: 'hardware', title: 'Hardware', sub: 'Your arms' },
  { n: 4, key: 'verify', title: 'Verify', sub: 'Configuration' },
  { n: 5, key: 'planning', title: 'Planning', sub: 'Continue' },
];

// ------------------------------------------------------------------ shared: setting the type
function useApplyRobotType() {
  const dispatch = useDispatch();
  const { setRobotType } = useRosServiceCaller();
  const phase = useSelector((s) => s.tasks.taskStatus.phase);
  return useCallback(
    async (type) => {
      if (phase > TaskPhase.READY) throw new Error('A task is running - the robot type cannot change now.');
      const result = await setRobotType(type);
      if (!(result && result.success)) throw new Error((result && result.message) || 'The robot server refused the robot type.');
      dispatch(selectRobotType(type));
      dispatch(setIsFirstLoadTrue('record'));
      dispatch(removeAllTags());
    },
    [setRobotType, dispatch, phase]
  );
}

function useRobotTypes() {
  const dispatch = useDispatch();
  const list = useSelector((s) => s.ui.robotTypeList);
  const { getRobotTypeList } = useRosServiceCaller();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const reload = useCallback(async () => {
    setLoading(true);
    setError('');
    try {
      const r = await getRobotTypeList();
      if (r && r.robot_types) dispatch(setRobotTypeList(r.robot_types));
      else setError('The robot server returned no robot types.');
    } catch (e) {
      setError(/timeout|connection/i.test(e.message) ? 'The robot server is not reachable (rosbridge on port 9090).' : e.message);
    } finally {
      setLoading(false);
    }
  }, [getRobotTypeList, dispatch]);
  return { list, loading, error, reload };
}

// ------------------------------------------------------------------ stepper
function Stepper({ step, kind, done, onGo }) {
  return (
    <ol className="grid grid-cols-5 gap-2" data-setup-stepper>
      {STEPS.map((s) => {
        const skipped = s.n === 2 && kind === 'builtin' && step > 1;
        const current = s.n === step;
        const isDone = done[s.n] && !current && !skipped;
        const reachable = s.n < step || done[s.n - 1] || s.n === 1;
        return (
          <li key={s.n}>
            <button
              type="button"
              disabled={!reachable || skipped}
              onClick={() => onGo(s.n)}
              data-setup-step={s.n}
              data-step-state={current ? 'current' : skipped ? 'skipped' : isDone ? 'done' : 'todo'}
              aria-current={current ? 'step' : undefined}
              className={clsx(
                'w-full rounded-2xl px-3 py-2.5 text-left transition-colors',
                current ? 'bg-ink text-white shadow-lift' : 'bg-paper hover:bg-paper-2',
                (skipped || !reachable) && 'opacity-55 hover:bg-paper cursor-default'
              )}
            >
              <div className="flex items-center gap-2">
                <span
                  className={clsx(
                    'flex h-5 w-5 shrink-0 items-center justify-center rounded-full text-[10.5px] font-semibold',
                    current ? 'bg-white text-ink' : isDone ? 'bg-blue-600 text-white' : 'bg-white text-ink-3'
                  )}
                >
                  {isDone ? '✓' : s.n}
                </span>
                <span className="text-[13px] font-semibold truncate">{s.title}</span>
              </div>
              <div className={clsx('mt-0.5 pl-7 text-[11px] truncate', current ? 'text-white/70' : 'text-ink-3')}>
                {skipped ? 'Not needed' : s.sub}
              </div>
            </button>
          </li>
        );
      })}
    </ol>
  );
}

function Footer({ onBack, back = 'Back', children }) {
  return (
    <div className="mt-5 flex items-center justify-between gap-2 border-t border-rule pt-4">
      {onBack ? (
        <button className="btn" onClick={onBack} data-action="setup-back">
          ← {back}
        </button>
      ) : (
        <span />
      )}
      <div className="flex items-center gap-2">{children}</div>
    </div>
  );
}

// ------------------------------------------------------------------ 1 · robot
function StepRobot({ view, setView, customs, reloadCustoms, onCancel }) {
  const types = useRobotTypes();
  const applyType = useApplyRobotType();
  const active = useSelector((s) => s.tasks.taskStatus.robotType);
  const [pick, setPick] = useState(view.robotType ? { kind: 'builtin', type: view.robotType } : view.customId ? { kind: 'custom', id: view.customId } : null);
  const [busy, setBusy] = useState(false);
  const { reload } = types;

  useEffect(() => {
    if (!types.list.length) reload();
    reloadCustoms();
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  // server robot types generated from a custom profile are listed with that profile
  const customKeys = new Set(customs.map((r) => customKey(r.id)));
  const builtins = types.list.filter((t) => !customKeys.has(t));

  const next = async () => {
    if (!pick) return;
    if (pick.kind === 'new') {
      setView({ step: 2, kind: 'custom', customId: null, robotType: null });
      return;
    }
    if (pick.kind === 'custom') {
      setView({ step: 2, kind: 'custom', customId: pick.id, robotType: null });
      return;
    }
    setBusy(true);
    try {
      await applyType(pick.type);
      toast.success(`Robot set to ${pick.type}`);
      setView({ step: 3, kind: 'builtin', robotType: pick.type, customId: null });
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };

  const tile = (on) =>
    clsx('text-left rounded-2xl p-3.5 transition-all border', on ? 'border-blue-600 bg-blue-50 shadow-sm' : 'border-transparent bg-paper hover:bg-paper-2');

  return (
    <Sheet title="1 · Choose your robot" code="S1">
      <p className="text-[13px] text-ink-3 mb-4 max-w-3xl">
        Pick a robot the robot server already knows, open one of your custom robots, or add a new custom robot from its model file.
      </p>
      <div className="grid gap-5 lg:grid-cols-2">
        <div>
          <div className="flex items-center justify-between mb-2">
            <div className="label">Robots on the robot server</div>
            <button className="btn-ghost text-[11.5px]" onClick={reload} disabled={types.loading}>
              {types.loading ? 'Loading…' : 'Refresh'}
            </button>
          </div>
          {types.error && <Callout tone="warn">{types.error}</Callout>}
          <div className="grid gap-2 sm:grid-cols-2" role="radiogroup" aria-label="Robot">
            {builtins.map((t) => (
              <button
                key={t}
                type="button"
                role="radio"
                aria-checked={pick?.type === t}
                data-setup-robot={t}
                className={tile(pick?.type === t)}
                onClick={() => setPick({ kind: 'builtin', type: t })}
              >
                <div className="text-[13px] font-semibold truncate">{t}</div>
                <div className="text-[11px] text-ink-3">{t === active ? 'Active now' : 'Built-in robot'}</div>
              </button>
            ))}
            {!builtins.length && !types.loading && (
              <div className="sm:col-span-2 rounded-2xl bg-paper p-4 text-[12.5px] text-ink-3">No robot types from the robot server yet.</div>
            )}
          </div>
        </div>
        <div>
          <div className="label mb-2">Custom robots</div>
          <div className="grid gap-2 sm:grid-cols-2">
            {customs.map((r) => {
              const stage = r.state === 'registered' ? r.stage || 1 : 0;
              return (
                <button
                  key={r.id}
                  type="button"
                  role="radio"
                  aria-checked={pick?.id === r.id}
                  data-setup-custom={r.id}
                  className={tile(pick?.id === r.id)}
                  onClick={() => setPick({ kind: 'custom', id: r.id })}
                >
                  <div className="text-[13px] font-semibold truncate">{r.name}</div>
                  <div className="text-[11px] text-ink-3 truncate">{stage ? `Stage ${stage} · ${STAGE_NAMES[stage]}` : 'Draft'}</div>
                </button>
              );
            })}
            <button
              type="button"
              role="radio"
              aria-checked={pick?.kind === 'new'}
              data-setup-add-custom
              className={clsx(tile(pick?.kind === 'new'), 'border-dashed !border-paper-3', pick?.kind === 'new' && '!border-blue-600')}
              onClick={() => setPick({ kind: 'new' })}
            >
              <div className="text-[13px] font-semibold">+ Add custom robot</div>
              <div className="text-[11px] text-ink-3">Start from its URDF / Xacro</div>
            </button>
          </div>
        </div>
      </div>
      <Footer onBack={onCancel} back="Home">
        <button className="btn-primary" disabled={!pick || busy} onClick={next} data-action="setup-next">
          {busy && <Spinner className="border-paper border-t-transparent" />}
          {pick?.kind === 'builtin' ? `Use ${pick.type} →` : pick?.kind === 'new' ? 'Add its model file →' : 'Next →'}
        </button>
      </Footer>
    </Sheet>
  );
}

// ------------------------------------------------------------------ 2 · model file
const baseName = (files) => {
  const main = files.find((f) => /\.(urdf|xacro)$/i.test(f.name)) || files.find((f) => /\.zip$/i.test(f.name)) || files[0];
  return (main ? main.name : 'Custom robot').replace(/\.(urdf\.xacro|urdf|xacro|zip|xml)$/i, '').replace(/[_-]+/g, ' ').trim() || 'Custom robot';
};
const uniqueName = (name, taken) => {
  const set = new Set(taken.map((t) => t.toLowerCase()));
  if (!set.has(name.toLowerCase())) return name;
  for (let i = 2; i < 100; i += 1) if (!set.has(`${name} ${i}`.toLowerCase())) return `${name} ${i}`;
  return `${name} ${Date.now()}`;
};

function UploadModel({ customs, onCreated }) {
  const [files, setFiles] = useState([]);
  const [busy, setBusy] = useState(false);
  const [progress, setProgress] = useState(null);
  const [error, setError] = useState('');
  const hasDesc = files.some((f) => /\.(zip|urdf|xacro)$/i.test(f.name));

  const submit = async () => {
    setError('');
    if (!hasDesc) {
      setError('Add the robot model: a .urdf or .xacro file (with its meshes), or a ZIP of the robot package.');
      return;
    }
    setBusy(true);
    let created = null;
    try {
      const taken = customs.map((r) => r.name);
      created = await createRobot({ name: uniqueName(baseName(files), taken), description: '', manufacturer: '', type: 'arm' });
      const r = await uploadAndValidate(created.id, files, setProgress);
      // the name inside the URDF (<robot name="...">) is a better default than the file name
      const urdfName = r.profile?.validation?.summary?.name;
      if (urdfName && urdfName !== created.name) {
        try {
          await updateRobot(created.id, { name: uniqueName(urdfName, taken) });
        } catch (e) {
          /* keep the file name */
        }
      }
      toast.success(r.needsEntry ? 'Uploaded - choose the main URDF file' : 'Model uploaded and checked');
      onCreated(created.id);
    } catch (e) {
      setError(e.message);
      if (created) {
        toast.error('The draft was saved; you can finish it here or in the robot library.');
        onCreated(created.id);
      }
    } finally {
      setBusy(false);
      setProgress(null);
    }
  };

  return (
    <div className="grid gap-5 lg:grid-cols-[1.3fr_1fr]">
      <div className="space-y-3">
        <FilePicker onFiles={(f) => setFiles((old) => [...old, ...f])} disabled={busy} />
        {files.length > 0 && (
          <div className="rounded-2xl bg-paper px-3 py-2 text-[12px] max-h-[150px] overflow-y-auto scroll-thin" data-selected-files>
            <div className="flex items-center justify-between mb-1">
              <span className="label">{files.length} file(s) selected</span>
              <button className="btn-ghost text-[11.5px]" onClick={() => setFiles([])} disabled={busy}>
                Clear
              </button>
            </div>
            {files.slice(0, 40).map((f, i) => (
              <div key={`${f.name}-${i}`} className="font-mono truncate">
                {f.webkitRelativePath || f.name} <span className="text-ink-4">· {(f.size / 1024).toFixed(0)} KB</span>
              </div>
            ))}
            {files.length > 40 && <div className="text-ink-3">… and {files.length - 40} more</div>}
          </div>
        )}
        {files.length > 0 && !hasDesc && (
          <Callout tone="warn">No .zip, .urdf or .xacro selected yet - meshes alone cannot be checked.</Callout>
        )}
        {error && (
          <div className="rounded-xl bg-signal-errbg px-3 py-2 text-[12.5px]" data-setup-error>
            {error}
          </div>
        )}
        <div className="flex items-center justify-end gap-2">
          {progress && (
            <span className="text-[12px] text-ink-3 mr-auto">
              Uploading {Math.min(progress.i + 1, progress.n)} / {progress.n} {progress.name}
            </span>
          )}
          <button className="btn-primary" onClick={submit} disabled={busy || !files.length} data-action="setup-upload">
            {busy && <Spinner className="border-paper border-t-transparent" />}
            Upload &amp; check model
          </button>
        </div>
      </div>
      <Callout tone="info" title="What happens">
        The files are checked on this PC and nothing in them is run; Xacro is expanded by a restricted built-in expander. The robot&apos;s
        name is taken from the model, and you get a validation report and a 3D view. You can add a description and manufacturer
        afterwards. Nothing moves.
      </Callout>
    </div>
  );
}

function DetailsForm({ robot, onSaved }) {
  const [meta, setMeta] = useState({ name: robot.name, manufacturer: robot.manufacturer || '', type: robot.type || 'arm', description: robot.description || '' });
  const [busy, setBusy] = useState(false);
  useEffect(() => {
    setMeta({ name: robot.name, manufacturer: robot.manufacturer || '', type: robot.type || 'arm', description: robot.description || '' });
  }, [robot.id, robot.name, robot.manufacturer, robot.type, robot.description]);
  const dirty =
    meta.name !== robot.name ||
    meta.manufacturer !== (robot.manufacturer || '') ||
    meta.type !== (robot.type || 'arm') ||
    meta.description !== (robot.description || '');
  const save = async () => {
    setBusy(true);
    try {
      await updateRobot(robot.id, meta);
      toast.success('Details saved');
      onSaved();
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  const set = (k) => (v) => setMeta((m) => ({ ...m, [k]: v }));
  return (
    <div className="space-y-2.5" data-setup-details>
      <div className="label">Details (optional)</div>
      <Field label="Name">
        <TextInput value={meta.name} onChange={set('name')} maxLength={80} data-field="name" />
      </Field>
      <div className="grid grid-cols-2 gap-2.5">
        <Field label="Manufacturer">
          <TextInput value={meta.manufacturer} onChange={set('manufacturer')} maxLength={80} data-field="manufacturer" />
        </Field>
        <Field label="Robot type">
          <Select
            value={meta.type}
            onChange={set('type')}
            options={Object.entries(ROBOT_TYPE_LABELS).map(([value, label]) => ({ value, label }))}
          />
        </Field>
      </div>
      <Field label="Description">
        <textarea className="input min-h-[56px]" value={meta.description} maxLength={1000} onChange={(e) => set('description')(e.target.value)} />
      </Field>
      <div className="flex justify-end">
        <button className="btn btn-sm" disabled={!dirty || busy || !meta.name.trim()} onClick={save} data-action="setup-save-details">
          Save details
        </button>
      </div>
    </div>
  );
}

function StepModel({ view, setView, customs, reloadCustoms }) {
  const [robot, setRobot] = useState(null);
  const [urdf, setUrdf] = useState(null);
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [showReport, setShowReport] = useState(false);
  const [replacing, setReplacing] = useState(false);
  const id = view.customId;

  const load = useCallback(async () => {
    if (!id) return;
    try {
      const r = await getRobot(id);
      setRobot(r);
      setError('');
      if (r.validation?.summary?.links) {
        try {
          setUrdf(await fetchResolvedUrdf(id));
        } catch (e) {
          setUrdf(null);
        }
      } else setUrdf(null);
    } catch (e) {
      setError(e.message);
    }
  }, [id]);

  useEffect(() => {
    setRobot(null);
    setUrdf(null);
    setReplacing(false);
    load();
  }, [load]);

  const back = () => setView({ step: 1 });

  if (!id || replacing) {
    return (
      <Sheet title={replacing ? '2 · Replace the model file' : '2 · Upload the robot model file'} code="S2">
        <p className="text-[13px] text-ink-3 mb-4 max-w-3xl">
          Start with the robot&apos;s description: a <span className="font-mono">.urdf</span> or <span className="font-mono">.xacro</span>{' '}
          file with its meshes, or a ZIP of the robot&apos;s package.
        </p>
        <UploadModel
          customs={replacing ? customs.filter((r) => r.id !== id) : customs}
          onCreated={(newId) => {
            reloadCustoms();
            setReplacing(false);
            setView({ customId: newId });
          }}
        />
        <Footer onBack={replacing ? () => setReplacing(false) : back} back={replacing ? 'Cancel' : 'Back'} />
      </Sheet>
    );
  }
  if (error && !robot) {
    return (
      <Sheet title="2 · Model file" code="S2">
        <Callout tone="err" title="Could not load this robot">{error}</Callout>
        <Footer onBack={back} />
      </Sheet>
    );
  }
  if (!robot) {
    return (
      <div className="p-8 flex justify-center">
        <Spinner />
      </div>
    );
  }

  const v = robot.validation;
  const s = v?.summary || {};
  const ok = v && v.status !== 'errors';
  const registered = robot.state === 'registered';
  const next = async () => {
    setBusy(true);
    try {
      if (!registered) {
        await registerRobot(robot.id);
        toast.success('Model registered (Stage 1)');
        reloadCustoms();
      }
      setView({ step: 3 });
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };

  return (
    <Sheet
      title={`2 · Model file · ${robot.name}`}
      code="S2"
      actions={<StageBadge stage={robot.stage} state={robot.state} />}
    >
      <div className="grid gap-5 lg:grid-cols-[1.5fr_1fr]" data-setup-model={robot.id}>
        <div className="space-y-3">
          {urdf ? (
            <UrdfViewer robotId={robot.id} urdfText={urdf} meshMap={s.mesh_map || {}} tree={v.tree} />
          ) : (
            <div className="rounded-2xl bg-paper py-16 text-center text-[13px] text-ink-3">The 3D view appears once the model has been read.</div>
          )}
        </div>
        <div className="space-y-4">
          {robot.entry_candidates && !robot.entry && (
            <Callout tone="warn" title="Several model files: choose the main one">
              <div className="flex flex-col gap-1 mt-1">
                {robot.entry_candidates.map((c) => (
                  <button
                    key={c}
                    className="btn btn-sm justify-start font-mono"
                    onClick={async () => {
                      try {
                        await validateRobot(robot.id, c);
                        load();
                      } catch (e) {
                        toast.error(e.message);
                      }
                    }}
                  >
                    {c}
                  </button>
                ))}
              </div>
            </Callout>
          )}
          {v && (
            <div className="rounded-2xl bg-paper p-3.5" data-setup-validation={v.status}>
              <div className="flex items-center justify-between">
                <div className="label">Model check</div>
                <span className={v.status === 'errors' ? 'chip-err' : v.status === 'warnings' ? 'chip-warn' : 'chip-ok'}>{v.status}</span>
              </div>
              <div className="mt-2 grid grid-cols-3 gap-2 text-center">
                {[
                  ['Links', s.links],
                  ['Joints', s.joints],
                  ['Moving', s.dof],
                ].map(([k, n]) => (
                  <div key={k} className="rounded-xl bg-white px-2 py-1.5">
                    <div className="text-[15px] font-semibold">{n ?? '-'}</div>
                    <div className="text-[10.5px] text-ink-3">{k}</div>
                  </div>
                ))}
              </div>
              <div className="mt-2 text-[12px] text-ink-3">
                Main file <span className="font-mono text-ink">{robot.entry || '-'}</span>
                {s.meshes ? ` · ${s.meshes_found || 0}/${s.meshes} meshes found` : ''}
              </div>
              <div className="mt-2 flex gap-2">
                <button className="btn-ghost text-[11.5px]" onClick={() => setShowReport((x) => !x)} data-action="setup-toggle-report">
                  {showReport ? 'Hide report' : 'Full report'}
                </button>
                <button className="btn-ghost text-[11.5px]" onClick={() => setReplacing(true)} disabled={registered} title={registered ? 'Registered models are replaced from the robot library (it resets the later stages)' : ''}>
                  Upload different files
                </button>
              </div>
            </div>
          )}
          {!v && !robot.entry_candidates && <Callout tone="warn">The model has not been checked yet.</Callout>}
          <DetailsForm robot={robot} onSaved={() => { load(); reloadCustoms(); }} />
        </div>
      </div>
      {showReport && v && (
        <div className="mt-4">
          <ValidationReport report={v} />
        </div>
      )}
      {v?.status === 'errors' && (
        <div className="mt-4">
          <Callout tone="err" title="Fix the errors in the model to continue">
            Open the full report for the suggested fixes, then upload the corrected files.
          </Callout>
        </div>
      )}
      <Footer onBack={back}>
        <button className="btn-primary" disabled={!ok || busy} onClick={next} data-action="setup-next">
          {busy && <Spinner className="border-paper border-t-transparent" />}
          {registered ? 'Next: hardware →' : 'Register model & continue →'}
        </button>
      </Footer>
    </Sheet>
  );
}

// ------------------------------------------------------------------ 3 · hardware
function ModeChooser({ robotType }) {
  const dispatch = useDispatch();
  const status = useHardwareStatus();
  const [mode, setMode] = useState('');
  const [busy, setBusy] = useState(false);
  const current = status && status.robot_type === robotType ? status.mode : mode;
  const choose = async (m) => {
    setBusy(true);
    try {
      const st = await hwSetup(robotType, { mode: m });
      setMode(st.mode);
      if (status && status.robot_type === robotType) dispatch(setHardwareStatus(st));
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  useEffect(() => {
    if (status && status.robot_type === robotType) return;
    // a custom robot that is not the active robot type yet: read its saved choice
    hwStatus(robotType)
      .then((st) => setMode(st.mode))
      .catch(() => setMode(''));
  }, [robotType, status]);
  return <ArmChoice mode={current} onChange={choose} disabled={busy} />;
}

function StepHardware({ view, setView }) {
  const status = useHardwareStatus();
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const back = () => setView({ step: view.kind === 'custom' ? 2 : 1 });
  if (view.kind === 'custom') {
    const key = customKey(view.customId || '');
    return (
      <Sheet title="3 · Arms" code="S3">
        <ModeChooser robotType={key} />
        <p className="mt-3 text-[12px] text-ink-3">Real arms are checked through ROS in the next step.</p>
        <Footer onBack={back}>
          <button className="btn-primary" onClick={() => setView({ step: 4 })} data-action="setup-next">
            Next: verify →
          </button>
        </Footer>
      </Sheet>
    );
  }
  const sim = status?.mode === 'simulation';
  const used = status ? Object.entries(status.arms || {}).filter(([, a]) => a.state !== 'unused') : [];
  const ready = used.every(([, a]) => a.state === 'connected');
  return (
    <div className="grid gap-4">
      <Sheet title={`3 · Hardware for ${robotType || 'the robot'}`} code="S3">
        <Footer onBack={back}>
          {status && !sim && !ready && <span className="text-[12px] text-ink-3">You can continue; step 4 shows what is still missing.</span>}
          <button className="btn-primary" onClick={() => setView({ step: 4 })} data-action="setup-next">
            Next: verify →
          </button>
        </Footer>
      </Sheet>
      <HardwareSetup />
    </div>
  );
}

// ------------------------------------------------------------------ 4 · verify
const REQUIRED = {
  leader_follower: { cap: 'teleoperation', text: 'Leader and follower both connected (teleoperation possible)' },
  leader_only: { cap: 'leader_monitor', text: 'Leader arm connected' },
  leader_sim: { cap: 'leader_monitor', text: 'Leader arm connected (the follower is simulated)' },
  follower_only: { cap: 'follower_monitor', text: 'Follower arm connected' },
  simulation: { cap: 'simulation', text: 'Simulation Only: no arm is needed' },
};

function CheckRow({ check }) {
  const tone = { ok: 'chip-ok', warn: 'chip-warn', fail: 'chip-err', skip: 'chip' }[check.state];
  const mark = { ok: '✓', warn: '!', fail: '✕', skip: '–' }[check.state];
  return (
    <li className="flex items-start gap-3 rounded-2xl bg-paper px-3.5 py-2.5" data-check={check.key} data-check-state={check.state}>
      <span className={clsx(tone, 'mt-0.5 w-6 justify-center text-[11px]')}>{mark}</span>
      <div className="min-w-0 flex-1">
        <div className="text-[13px] font-medium">{check.title}</div>
        {check.detail && <div className="text-[12px] text-ink-3 mt-0.5">{check.detail}</div>}
      </div>
      {check.action && (
        <button className="btn btn-sm shrink-0" onClick={check.action.run}>
          {check.action.label}
        </button>
      )}
    </li>
  );
}

function builtinChecks({ robotType, viewType, heartbeat, status, offline, goHardware }) {
  const out = [];
  const typeOk = robotType && robotType === viewType;
  out.push({
    key: 'robot_type',
    title: typeOk ? `Robot server uses ${robotType}` : 'Robot server has not confirmed the robot',
    detail: typeOk
      ? heartbeat === 'connected'
        ? 'The robot server answers (heartbeat alive).'
        : 'The robot type is set, but the robot server heartbeat is not alive right now.'
      : 'Go back to step 1 and choose the robot again.',
    state: typeOk ? (heartbeat === 'connected' ? 'ok' : 'warn') : 'fail',
    blocking: !typeOk,
  });
  if (offline || !status) {
    out.push({
      key: 'hardware',
      title: 'Hardware status is not available',
      detail: 'The RawBot Studio training service is not reachable, so the arms cannot be checked here. Everything works as before.',
      state: 'warn',
    });
    return out;
  }
  const req = REQUIRED[status.mode] || REQUIRED.leader_follower;
  const cap = status.capabilities?.[req.cap];
  const modeLabel = (MODE_OPTIONS.find((m) => m.value === status.mode) || {}).label;
  out.push({ key: 'mode', title: `Configuration: ${modeLabel}`, detail: 'Change it in step 3.', state: 'ok' });
  if (status.mode === 'simulation') {
    out.push({ key: 'arms', title: req.text, state: 'skip' });
    return out;
  }
  out.push({
    key: 'arms',
    title: cap?.available ? req.text : `Not ready: ${req.text.toLowerCase()}`,
    detail: cap?.available ? '' : cap?.reason,
    state: cap?.available ? 'ok' : 'warn',
    action: cap?.available ? null : { label: 'Fix in step 3', run: goHardware },
  });
  Object.entries(status.arms || {}).forEach(([role, a]) => {
    if (a.state === 'unused') return;
    const name = role === 'leader' ? 'Leader' : 'Follower';
    const cal = a.calibration || {};
    out.push({
      key: `${role}_calibration`,
      title: `${name} calibration: ${cal.state === 'calibrated' ? 'calibrated' : cal.state === 'factory' ? 'not calibrated' : 'not checked'}`,
      detail:
        cal.state === 'calibrated'
          ? cal.text
          : cal.state === 'factory'
            ? 'Calibrate this arm in step 3 before recording.'
            : `${STATE_LABEL[a.state] || a.state}. The calibration is read when RawBot Studio scans the motors (bridge.py stopped).`,
      state: cal.state === 'calibrated' ? 'ok' : 'warn',
    });
  });
  return out;
}

function CustomVerify({ customId, onConfirmed, footer }) {
  const [robot, setRobot] = useState(null);
  const [tab, setTab] = useState('config');
  const [busy, setBusy] = useState(false);
  const types = useRobotTypes();
  const applyType = useApplyRobotType();
  const active = useSelector((s) => s.tasks.taskStatus.robotType);
  const load = useCallback(async () => {
    try {
      setRobot(await getRobot(customId));
    } catch (e) {
      toast.error(e.message);
    }
  }, [customId]);
  useEffect(() => {
    load();
  }, [load]);
  if (!robot) {
    return (
      <div className="p-8 flex justify-center">
        <Spinner />
      </div>
    );
  }
  const stage = robot.state === 'registered' ? robot.stage || 1 : 0;
  const key = customKey(robot.id);
  const onServer = types.list.includes(key);
  const confirmed = active === key;
  const extra = robotExtraTabs.find((t) => t.value === tab);
  const useIt = async () => {
    setBusy(true);
    try {
      await applyType(key);
      toast.success(`Robot set to ${robot.name}`);
      onConfirmed(key);
    } catch (e) {
      toast.error(e.message);
    } finally {
      setBusy(false);
    }
  };
  // a robot RawBot Studio never commands (command interface none: a leader moved by hand) needs
  // no ROS follower check or motion test before it is used
  const ro = Boolean(robot.read_only || robot.readiness?.read_only);
  const checks = [
    { key: 'model', title: 'Model registered', detail: stage >= 1 ? 'Stage 1 · URDF registered' : 'Register the model in step 2.', state: stage >= 1 ? 'ok' : 'fail' },
    { key: 'config', title: 'Robot configuration saved', detail: robot.config ? 'Joints, topics and units are set.' : 'Fill in the Robot configuration below and save it.', state: robot.config ? 'ok' : 'fail' },
    ro && stage < 2
      ? { key: 'ros', title: 'ROS checks (Stage 2)', detail: 'Optional: RawBot Studio only reads this robot.', state: 'ok' }
      : { key: 'ros', title: 'ROS checks (Stage 2)', detail: stage >= 2 ? 'Topics and joints match the model.' : 'Run the ROS checks under Robot readiness.', state: stage >= 2 ? 'ok' : 'fail' },
    ro && stage < 3
      ? { key: 'hw', title: 'Hardware verified (Stage 3)', detail: 'Not needed: no commands are sent to this robot (command interface: none).', state: 'ok' }
      : { key: 'hw', title: 'Hardware verified (Stage 3)', detail: stage >= 3 ? 'The supervised one-joint test passed.' : 'Run the supervised hardware test under Robot readiness.', state: stage >= 3 ? 'ok' : 'fail' },
    {
      key: 'server',
      title: onServer ? 'Known to the robot server' : 'Not on the robot server yet',
      detail: onServer
        ? confirmed
          ? 'This robot is the active robot type.'
          : 'Use it as the active robot type to continue.'
        : 'After Stage 3: Export server config (Robot readiness), save it in data/robot-config/, restart the robot runtime, then press Refresh.',
      state: onServer ? (confirmed ? 'ok' : 'warn') : 'fail',
      action: onServer && !confirmed && (stage >= 3 || ro) ? { label: busy ? 'Setting…' : 'Use this robot', run: useIt } : !onServer ? { label: 'Refresh', run: types.reload } : null,
    },
  ];
  return (
    <div className="grid gap-4">
      <Sheet title={`4 · Verify ${robot.name}`} code="S4" actions={<StageBadge stage={robot.stage} state={robot.state} />}>
        <ul className="grid gap-2" data-setup-checks>
          {checks.map((c) => (
            <CheckRow key={c.key} check={c} />
          ))}
        </ul>
        {footer}
      </Sheet>
      <Segmented size="sm" value={tab} onChange={setTab} options={robotExtraTabs.map((t) => ({ value: t.value, label: t.label }))} />
      {extra && extra.render({ robot, refresh: load })}
    </div>
  );
}

function StepVerify({ view, setView }) {
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const heartbeat = useSelector((s) => s.tasks.heartbeatStatus);
  const offline = useSelector((s) => s.hardware.offline);
  const status = useHardwareStatus();
  const [stamp, setStamp] = useState(0);
  const back = () => setView({ step: 3 });

  if (view.kind === 'custom') {
    const key = customKey(view.customId || '');
    const ok = robotType === key;
    return (
      <div className="grid gap-4">
        <CustomVerify
          customId={view.customId}
          onConfirmed={(k) => setView({ robotType: k })}
          footer={
            <Footer onBack={back}>
              {!ok && <span className="text-[12px] text-ink-3">Planning starts once this robot is the active robot type.</span>}
              <button className="btn-primary" disabled={!ok} onClick={() => setView({ step: 5, verified: true })} data-action="setup-next">
                Next →
              </button>
            </Footer>
          }
        />
      </div>
    );
  }

  const hwStatus = status && status.robot_type === robotType ? status : null;
  const checks = builtinChecks({ robotType, viewType: view.robotType, heartbeat, status: hwStatus, offline, goHardware: () => setView({ step: 3 }) });
  const blocking = checks.some((c) => c.blocking);
  const warnings = checks.filter((c) => c.state === 'warn' || c.state === 'fail').length;
  return (
    <Sheet
      title={`4 · Verify ${robotType || ''}`}
      code="S4"
      actions={
        <button className="btn btn-sm" onClick={() => setStamp(stamp + 1)} data-action="setup-recheck">
          Check again
        </button>
      }
    >
      <p className="text-[13px] text-ink-3 mb-4 max-w-3xl">
        Live checks of the robot server, the configuration and each arm. Warnings do not stop you from planning (planning never moves the
        robot); they tell you what to fix before recording.
      </p>
      <ul className="grid gap-2" data-setup-checks key={stamp}>
        {checks.map((c) => (
          <CheckRow key={c.key} check={c} />
        ))}
      </ul>
      <Footer onBack={back}>
        {blocking ? (
          <span className="text-[12px] text-signal-err">Fix the failed check to continue.</span>
        ) : warnings ? (
          <span className="text-[12px] text-ink-3">{warnings} item(s) to look at before recording.</span>
        ) : (
          <span className="text-[12px] text-signal-ok">All checks passed.</span>
        )}
        <button className="btn-primary" disabled={blocking} onClick={() => setView({ step: 5, verified: !warnings })} data-action="setup-next">
          {warnings && !blocking ? 'Continue with warnings →' : 'Next →'}
        </button>
      </Footer>
    </Sheet>
  );
}

// ------------------------------------------------------------------ 5 · planning
function StepPlanning({ view, setView, customs }) {
  const dispatch = useDispatch();
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const status = useHardwareStatus();
  const custom = customs.find((r) => r.id === view.customId);
  const mode = status && status.robot_type === robotType ? (MODE_OPTIONS.find((m) => m.value === status.mode) || {}).label : '-';
  const rows = [
    ['Robot', custom ? custom.name : robotType],
    ['Model', custom ? `Custom · ${custom.entry || 'uploaded'}` : 'Built-in (from the robot server)'],
    ['Hardware', mode],
    ['Verification', view.verified ? 'All checks passed' : 'Passed with warnings'],
  ];
  return (
    <Sheet title="5 · Ready for planning" code="S5">
      <div className="grid gap-5 lg:grid-cols-[1fr_1.2fr]">
        <dl className="grid gap-2" data-setup-summary>
          {rows.map(([k, v]) => (
            <div key={k} className="flex items-center justify-between rounded-2xl bg-paper px-3.5 py-2.5">
              <dt className="text-[12px] text-ink-3">{k}</dt>
              <dd className="text-[13px] font-medium">{v}</dd>
            </div>
          ))}
        </dl>
        <div className="rounded-2xl bg-paper p-4 text-[13px] text-ink-2">
          <div className="font-semibold text-ink">Next: the Planning session</div>
          <p className="mt-1">
            Describe the task, place the objects and generate a collection plan. Planning works on the model only and never moves the robot.
            You can come back to this setup any time from Home or Robots.
          </p>
        </div>
      </div>
      <Footer onBack={() => setView({ step: 4 })}>
        <button className="btn-primary" disabled={!robotType} onClick={() => dispatch(moveToPage(PageType.PLANNING))} data-action="setup-to-planning">
          Continue to Planning →
        </button>
      </Footer>
    </Sheet>
  );
}

// ------------------------------------------------------------------ the flow
export default function SetupFlow() {
  const dispatch = useDispatch();
  const stored = useSelector((s) => s.ui.robotsView);
  const view = useMemo(() => stored || {}, [stored]);
  const [customs, setCustoms] = useState([]);
  const step = view.step || 1;
  const kind = view.kind || (view.customId ? 'custom' : 'builtin');

  const setView = useCallback((patch) => dispatch(setRobotsView({ ...view, mode: 'setup', ...patch })), [dispatch, view]);
  const reloadCustoms = useCallback(() => {
    listRobots()
      .then((r) => setCustoms(r.robots || []))
      .catch(() => setCustoms([]));
  }, []);
  useEffect(() => {
    reloadCustoms();
  }, [reloadCustoms]);

  const done = useMemo(
    () => ({
      1: step > 1,
      2: kind === 'builtin' ? step > 1 : step > 2,
      3: step > 3,
      4: step > 4,
    }),
    [step, kind]
  );

  const cancel = () => dispatch(moveToPage(PageType.HOME));
  const props = { view: { ...view, kind }, setView, customs, reloadCustoms };
  return (
    <div className="grid gap-4" data-setup-flow data-setup-at={step}>
      <Stepper step={step} kind={kind} done={done} onGo={(n) => setView({ step: n })} />
      {step === 1 && <StepRobot {...props} onCancel={cancel} />}
      {step === 2 && <StepModel {...props} />}
      {step === 3 && <StepHardware {...props} />}
      {step === 4 && <StepVerify {...props} />}
      {step === 5 && <StepPlanning {...props} />}
    </div>
  );
}
