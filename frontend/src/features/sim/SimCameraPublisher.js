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

// Simulated cameras. When the simulated robot is the recorded robot and a camera of the robot
// configuration has no real publisher, the hardware status lists it in planning.sim_cameras.
// This renders the simulated robot - built from its kinematic chain (URDF joints, or the SO-101
// chain), so any robot works - and publishes JPEG CompressedImage frames on that camera topic
// at 10 Hz. 'scene' cameras look at the table from the front, 'wrist' cameras sit on the tool.
// Fixed light palette: frames must not depend on the page theme. No UI.

import { useEffect, useRef } from 'react';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import ROSLIB from 'roslib';
import rosConnectionManager from '../../utils/rosConnectionManager';
import { useHardwareStatus } from '../hardware/useHardwareGate';
import { SIM_TOPIC } from './simTopics';

const W = 320;
const H = 240;
const FPS = 10;

function buildRobot(chain) {
  // one group per joint: fixed origin (xyz, rpy), then the joint's own motion
  const root = new THREE.Group();
  const moving = {};
  const linkMat = new THREE.MeshLambertMaterial({ color: 0x2457e6 });
  const jointMat = new THREE.MeshLambertMaterial({ color: 0x121414 });
  let parent = root;
  (chain.joints || []).forEach((j) => {
    const origin = new THREE.Group();
    const [x, y, z] = j.xyz || [0, 0, 0];
    origin.position.set(x, y, z);
    const [r, p, w] = j.rpy || [0, 0, 0];
    origin.rotation.set(r, p, w, 'ZYX');
    // the link from the parent frame to this joint
    const len = Math.hypot(x, y, z);
    if (len > 1e-4) {
      const link = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, len, 10), linkMat);
      link.position.set(x / 2, y / 2, z / 2);
      link.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(x, y, z).normalize());
      parent.add(link);
    }
    const motion = new THREE.Group();
    origin.add(motion);
    if (j.type !== 'fixed') origin.add(new THREE.Mesh(new THREE.SphereGeometry(0.016, 12, 8), jointMat));
    parent.add(origin);
    moving[(chain.names || {})[j.name] || j.name] = { j, motion };
    parent = motion;
  });
  const tool = new THREE.Group();
  parent.add(tool);
  return { root, moving, tool };
}

function pose(robot, names, positions) {
  names.forEach((n, i) => {
    const m = robot.moving[n];
    if (!m) return;
    const v = Number(positions[i]) || 0;
    const ax = new THREE.Vector3(...(m.j.axis || [1, 0, 0])).normalize();
    if (m.j.type === 'prismatic') m.motion.position.copy(ax.multiplyScalar(v));
    else if (m.j.type !== 'fixed') m.motion.quaternion.setFromAxisAngle(ax, v);
  });
}

export default function SimCameraPublisher() {
  const status = useHardwareStatus();
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const cams = status?.planning?.sim_cameras || [];
  const chain = status?.sim?.chain;
  const key = cams.map((c) => `${c.topic}:${c.view}`).join(',');
  const chainKey = chain ? JSON.stringify(chain.joints.map((j) => j.name)) : '';
  const chainRef = useRef(chain);
  chainRef.current = chain;

  useEffect(() => {
    if (!key || !chainKey) return undefined;
    const list = key.split(',').map((s) => {
      const i = s.lastIndexOf(':');
      return { topic: s.slice(0, i), view: s.slice(i + 1) };
    });
    let alive = true;
    let timer = null;
    let sub = null;
    const joints = { names: [], positions: [] };
    const canvas = document.createElement('canvas');
    canvas.width = W;
    canvas.height = H;
    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    } catch (e) {
      return undefined; // no WebGL here: no simulated frames (the status keeps saying so)
    }
    renderer.setSize(W, H, false);
    renderer.setClearColor(0xf4f5f2);
    const scene = new THREE.Scene();
    scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const sun = new THREE.DirectionalLight(0xffffff, 0.8);
    sun.position.set(0.5, -0.6, 1.2);
    scene.add(sun);
    const table = new THREE.Mesh(new THREE.PlaneGeometry(1.2, 1.0), new THREE.MeshLambertMaterial({ color: 0xd8d2c4 }));
    table.position.set(0.25, 0, -0.001);
    scene.add(table);
    const grid = new THREE.GridHelper(1.0, 20, 0xb9b3a5, 0xc8c2b4);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0.25, 0, 0);
    scene.add(grid);
    const robot = buildRobot(chainRef.current);
    scene.add(robot.root);
    const sceneCam = new THREE.PerspectiveCamera(45, W / H, 0.01, 10);
    sceneCam.up.set(0, 0, 1);
    sceneCam.position.set(0.7, -0.45, 0.5);
    sceneCam.lookAt(0.18, 0, 0.05);
    const wristCam = new THREE.PerspectiveCamera(70, W / H, 0.005, 5);
    wristCam.up.set(0, 0, 1);
    robot.tool.add(wristCam);
    // the tool's +x looks forward; a camera looks down its own -z
    wristCam.rotation.set(0, -Math.PI / 2, -Math.PI / 2);
    wristCam.position.set(-0.03, 0, 0.03);

    rosConnectionManager
      .getConnection(rosbridgeUrl)
      .then((ros) => {
        if (!alive) return;
        sub = new ROSLIB.Topic({ ros, name: SIM_TOPIC, messageType: 'sensor_msgs/msg/JointState', throttle_rate: 50 });
        sub.subscribe((m) => {
          joints.names = m.name || [];
          joints.positions = m.position || [];
        });
        const pubs = list.map((c) => ({ ...c, pub: new ROSLIB.Topic({ ros, name: c.topic, messageType: 'sensor_msgs/msg/CompressedImage' }) }));
        timer = setInterval(() => {
          pose(robot, joints.names, joints.positions);
          const t = Date.now() / 1000;
          pubs.forEach((c) => {
            renderer.render(scene, c.view === 'wrist' ? wristCam : sceneCam);
            const data = canvas.toDataURL('image/jpeg', 0.85).split(',')[1];
            c.pub.publish({
              header: { stamp: { sec: Math.floor(t), nanosec: Math.floor((t % 1) * 1e9) }, frame_id: 'rawbot_sim' },
              format: 'jpeg',
              data,
            });
          });
        }, 1000 / FPS);
      })
      .catch(() => {});
    return () => {
      alive = false;
      if (timer) clearInterval(timer);
      if (sub) sub.unsubscribe();
      renderer.dispose();
    };
  }, [key, chainKey, rosbridgeUrl]);

  return null;
}
