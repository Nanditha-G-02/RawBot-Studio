// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0

// 3D view of the simulated robot in the shared world (simWorld.js): the robot's own URDF when
// it is a custom robot, otherwise its kinematic chain. The pose comes only from the live joint
// state topic (what is recorded); with no data the view says so and holds the last pose.
// Objects come from the simulated robot's objects topic (planned or physics poses). The two
// insets render the simulated cameras exactly as they are published.

import React, { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import clsx from 'clsx';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import ROSLIB from 'roslib';
import rosConnectionManager from '../../utils/rosConnectionManager';
import { STAGE, TABLE_H, createWorld, loadCustomModel, makeCameras, makeChainRobot, makeUrdfRobot, setupRenderer, viewPose } from './simWorld';
import useSimObjects from './useSimObjects';

const OBJECTS_FRESH_MS = 3000;

function sceneCameraRig(cam, far) {
  const g = new THREE.Group();
  const dark = new THREE.MeshStandardMaterial({ color: 0x2a2d31, roughness: 0.4 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.035, 0.03), dark);
  body.position.copy(cam.position);
  body.quaternion.copy(cam.quaternion);
  const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.012, 16), new THREE.MeshStandardMaterial({ color: 0x2f7fd6, roughness: 0.2, metalness: 0.4 }));
  lens.rotation.x = Math.PI / 2;
  lens.position.z = -0.02;
  body.add(lens);
  g.add(body);
  // a stand from the floor up to the camera (cameras are mounted, not floating)
  const h = cam.position.z + TABLE_H;
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, h, 12), dark);
  pole.rotation.x = Math.PI / 2;
  pole.position.set(cam.position.x, cam.position.y, cam.position.z - h / 2 - 0.02);
  g.add(pole);
  const f = new THREE.PerspectiveCamera(cam.fov, cam.aspect, 0.03, far);
  f.position.copy(cam.position);
  f.quaternion.copy(cam.quaternion);
  f.updateMatrixWorld();
  const helper = new THREE.CameraHelper(f);
  helper.setColors(...Array(5).fill(new THREE.Color(STAGE.camera)));
  g.add(f);
  g.userData.helper = helper;
  g.traverse((n) => {
    if (n.isMesh) n.castShadow = true;
  });
  return g;
}

function wristMarker(cam, far) {
  const g = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.026, 0.018, 0.02), new THREE.MeshStandardMaterial({ color: 0x2a2d31, roughness: 0.4 }));
  g.add(body);
  const h = Math.tan(THREE.MathUtils.degToRad(cam.fov / 2)) * far;
  const w = h * cam.aspect;
  const c = [[-w, -h, -far], [w, -h, -far], [w, h, -far], [-w, h, -far]];
  const pts = [];
  c.forEach((p, i) => pts.push(0, 0, 0, ...p, ...p, ...c[(i + 1) % 4]));
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  g.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: STAGE.camera, transparent: true, opacity: 0.7 })));
  return g;
}

export default function SimViewport({ robotType, topic, chain, items, cameras = ['scene', 'wrist'], className, height = 360, hud = true, caption }) {
  const mountRef = useRef(null);
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const jointsRef = useRef({ names: [], positions: [], stamp: 0, n: 0, t0: 0 });
  const itemsRef = useRef(items || []);
  itemsRef.current = items || [];
  const objects = useSimObjects(true);
  const [model, setModel] = useState(undefined); // undefined = loading, null = no custom model
  const [live, setLive] = useState({ age: null, hz: null });
  const [problem, setProblem] = useState('');
  const chainKey = chain ? JSON.stringify((chain.joints || []).map((j) => j.name)) : '';
  const chainRef = useRef(chain);
  chainRef.current = chain;
  const camKey = cameras.join(',');
  const labelsRef = useRef({});

  useEffect(() => {
    let alive = true;
    setModel(undefined);
    loadCustomModel(robotType).then((m) => alive && setModel(m));
    return () => {
      alive = false;
    };
  }, [robotType]);

  // live joints
  useEffect(() => {
    if (!topic) return undefined;
    let sub = null;
    let alive = true;
    rosConnectionManager
      .getConnection(rosbridgeUrl)
      .then((ros) => {
        if (!alive) return;
        sub = new ROSLIB.Topic({ ros, name: topic, messageType: 'sensor_msgs/msg/JointState', throttle_rate: 20 });
        sub.subscribe((m) => {
          const j = jointsRef.current;
          const now = performance.now();
          jointsRef.current = { names: m.name || [], positions: m.position || [], stamp: now, n: j.n + 1, t0: j.t0 || now };
        });
        setProblem('');
      })
      .catch((e) => setProblem(`Cannot subscribe to ${topic}: ${e.message}`));
    const t = setInterval(() => {
      const j = jointsRef.current;
      const now = performance.now();
      const hz = j.n > 5 && now > j.t0 ? (j.n * 1000) / (now - j.t0) : null;
      if (now - j.t0 > 5000 && j.n) jointsRef.current = { ...j, n: 0, t0: now };
      setLive({ age: j.stamp ? now - j.stamp : null, hz });
    }, 500);
    return () => {
      alive = false;
      clearInterval(t);
      if (sub) sub.unsubscribe();
    };
  }, [topic, rosbridgeUrl]);

  // the scene
  useEffect(() => {
    const el = mountRef.current;
    if (!el || model === undefined) return undefined;
    if (!model && !chainRef.current) return undefined;
    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch (e) {
      setProblem('WebGL is not available in this browser.');
      return undefined;
    }
    setupRenderer(renderer);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.autoClear = false;
    el.appendChild(renderer.domElement);

    const w = createWorld();
    let robot;
    try {
      robot = model ? makeUrdfRobot(model) : makeChainRobot(chainRef.current);
    } catch (e) {
      setProblem(e.message);
      renderer.dispose();
      el.removeChild(renderer.domElement);
      return undefined;
    }
    w.setRobot(robot.root);
    w.fitTable(robot.length);
    robot.ready.then(() => w.setRobot(robot.root));
    const cams = makeCameras(robot, w, 4 / 3);
    const L = robot.length;

    // camera rigs (hidden in the camera renders: they are not in the recorded images)
    const rig = sceneCameraRig(cams.scene, L * 0.18);
    w.world.add(rig);
    w.scene.add(rig.userData.helper);
    const wm = wristMarker(cams.wrist, L * 0.12);
    cams.wrist.add(wm);

    const vp = viewPose(robot);
    const view = new THREE.PerspectiveCamera(42, 1, 0.01, 80);
    view.up.set(0, 0, 1);
    w.world.add(view);
    view.position.set(...vp.position);
    const controls = new OrbitControls(view, renderer.domElement);
    controls.target.set(...vp.target);
    controls.enableDamping = true;
    controls.maxPolarAngle = Math.PI * 0.495;
    controls.minDistance = L * 0.3;
    controls.maxDistance = L * 12;
    // OrbitControls works in the scene frame (Y-up); the view camera lives in the Z-up world
    view.parent.updateMatrixWorld(true);
    const toScene = (v) => v.clone().applyMatrix4(w.world.matrixWorld);
    const camWorld = toScene(view.position);
    w.world.remove(view);
    w.scene.add(view);
    view.up.set(0, 1, 0);
    view.position.copy(camWorld);
    controls.target.copy(toScene(controls.target));
    controls.update();

    // the start view backs off in a narrow or tall panel, so the workspace fits its width;
    // once the user orbits or zooms, their view is kept
    const home = view.position.clone().sub(controls.target);
    let touched = false;
    // drawn only when something changed (joints, objects, the view, the size, meshes loaded):
    // a still robot costs nothing
    let dirty = true;
    controls.addEventListener('start', () => {
      touched = true;
    });
    const resize = () => {
      const width = el.clientWidth || 600;
      const h = el.clientHeight || height;
      renderer.setSize(width, h, false);
      renderer.domElement.style.width = '100%';
      renderer.domElement.style.height = '100%';
      view.aspect = width / h;
      view.updateProjectionMatrix();
      if (!touched) {
        view.position.copy(controls.target).add(home.clone().multiplyScalar(Math.max(1, Math.min(1.8, 2 / view.aspect)) ** 0.6));
        controls.update();
      }
      dirty = true;
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(el);

    const camNames = camKey.split(',').filter((k) => k && cams[k]);
    const insetCams = camNames.map((k) => cams[k]);
    let raf = 0;
    const size = new THREE.Vector2();
    // at most 30 frames per second: the joints arrive at 30 Hz, and the view draws the robot
    // three times per frame (view + two camera insets) plus shadows - heavy meshes on a
    // machine without a fast GPU must not take the page down with it
    // slow machine (software WebGL, no GPU): after 20 frames averaging over 60 ms, shadows go
    // and the pixel ratio drops to 1 (the camera images are unaffected: they are rendered
    // by SimCameraPublisher at their own size)
    let last = 0;
    let cost = 0;
    let frames = 0;
    let lite = false;
    let lastPos = [];
    let lastItems = '';
    const touch = () => {
      dirty = true;
    };
    controls.addEventListener('change', touch);
    robot.ready.then(touch);
    const loop = (now = performance.now()) => {
      raf = requestAnimationFrame(loop);
      if (now - last < 32) return;
      const j = jointsRef.current;
      if (j.positions.length !== lastPos.length || j.positions.some((v, i) => Math.abs(v - lastPos[i]) > 1e-5)) {
        lastPos = j.positions.slice();
        robot.pose(j.names, j.positions);
        dirty = true;
      }
      const o = objects.current;
      const items = o.stamp && performance.now() - o.stamp < OBJECTS_FRESH_MS ? o.items : itemsRef.current;
      const itemsKey = JSON.stringify(items);
      if (itemsKey !== lastItems) {
        lastItems = itemsKey;
        w.setItems(items);
        dirty = true;
      }
      controls.update();
      if (!dirty) return;
      dirty = false;
      last = now;
      frames += 1;
      el.dataset.frames = String(frames);
      if (!lite && frames > 20 && cost / frames > 60) {
        lite = true;
        renderer.shadowMap.enabled = false;
        w.scene.traverse((n) => {
          if (n.material) (Array.isArray(n.material) ? n.material : [n.material]).forEach((m) => (m.needsUpdate = true));
        });
        renderer.setPixelRatio(1);
        resize();
        el.dataset.quality = 'lite';
      }
      renderer.getSize(size);
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, size.x, size.y);
      renderer.clear();
      rig.visible = wm.visible = rig.userData.helper.visible = true;
      w.helpers.forEach((x) => {
        x.visible = true;
      });
      renderer.render(w.scene, view);
      // camera insets, bottom right, the scene camera above the wrist camera; as large as fits
      // (at most 220 px wide, 27% of the width, and all of them within the height)
      const n = insetCams.length;
      const ihMax = n ? (size.y - 10 - 22 * n) / n : 0;
      const iw = Math.max(0, Math.round(Math.min(220, size.x * 0.27, (ihMax * 4) / 3)));
      const ih = Math.round((iw * 3) / 4);
      rig.visible = wm.visible = rig.userData.helper.visible = false;
      w.helpers.forEach((x) => {
        x.visible = false;
      });
      renderer.setScissorTest(true);
      insetCams.forEach((c, i) => {
        const x = size.x - iw - 10;
        const y = 10 + (ih + 22) * (n - 1 - i);
        const label = labelsRef.current[camNames[i]];
        if (label) {
          label.style.bottom = `${y + ih + 3}px`;
          label.style.display = iw < 60 ? 'none' : '';
        }
        if (iw < 60) return;
        renderer.setViewport(x, y, iw, ih);
        renderer.setScissor(x, y, iw, ih);
        renderer.clear();
        renderer.render(w.scene, c);
      });
      renderer.setScissorTest(false);
      cost += performance.now() - now;
    };
    loop();
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      controls.dispose();
      w.dispose();
      renderer.dispose();
      if (renderer.domElement.parentNode === el) el.removeChild(renderer.domElement);
    };
  }, [model, chainKey, camKey, height, objects]);

  const state = live.age === null ? 'none' : live.age > 1000 ? 'stale' : 'live';
  const names = camKey.split(',').filter(Boolean);
  const mono = 'font-mono text-[11px] leading-[16px]';
  return (
    <div
      className={clsx('relative w-full overflow-hidden rounded-xl border border-rule', className)}
      style={{ height, background: '#2b2f35', containerType: 'inline-size' }}
      data-sim-viewport={model ? 'urdf' : model === null ? 'chain' : 'loading'}
      data-sim-live={state}
    >
      <div ref={mountRef} className="absolute inset-0" />
      {hud && (
        <div className="pointer-events-none absolute left-3 top-2.5">
          <div className={clsx(mono, 'text-[12px] text-[rgba(235,238,242,0.9)]')}>
            {(model && model.name) || robotType || 'Robot'} · {caption || 'simulated follower'}
          </div>
          <div className={clsx(mono, state === 'live' ? 'text-[rgba(235,238,242,0.55)]' : 'text-[#f0a35e]')}>
            {topic || '—'} · {state === 'live' ? `${live.hz ? Math.round(live.hz) : '…'} Hz` : state === 'stale' ? 'no new joint data' : 'no joint data'}
          </div>
        </div>
      )}
      <div className="pointer-events-none absolute inset-0">
        {names.map((n) => (
          <span
            key={n}
            className={clsx(mono, 'absolute text-[rgba(235,238,242,0.85)]')}
            ref={(e) => {
              labelsRef.current[n] = e;
            }}
            style={{ right: 10, bottom: 10 }}
          >
            {n} · sim
          </span>
        ))}
      </div>
      {model === undefined && <div className="absolute inset-0 grid place-items-center text-[12.5px] text-[rgba(235,238,242,0.7)]">Loading the robot model…</div>}
      {model === null && !chain && <div className="absolute inset-0 grid place-items-center text-[12.5px] text-[rgba(235,238,242,0.7)]">No model for this robot.</div>}
      {problem && <div className="absolute inset-x-2 bottom-2 border border-signal-err/40 bg-signal-errbg px-2 py-1 text-[12px]">{problem}</div>}
    </div>
  );
}
