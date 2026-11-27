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
// This renders the simulated robot in the shared world (simWorld.js: the same stage the 3D view
// shows) - its uploaded URDF for a custom robot, else its kinematic chain - and publishes JPEG
// CompressedImage frames on that camera topic at 10 Hz. 'scene' cameras look at the table from
// the front, 'wrist' cameras sit on the tool. The scene objects (plan episode) are drawn where
// the simulated robot's objects topic says - planned, or the physics result with MuJoCo - so
// the recorded images show the same world as the 3D view. Fixed look: frames never depend on
// the page theme.

import { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import * as THREE from 'three';
import ROSLIB from 'roslib';
import rosConnectionManager from '../../utils/rosConnectionManager';
import { useHardwareStatus } from '../hardware/useHardwareGate';
import { SIM_TOPIC } from './simTopics';
import { createWorld, loadCustomModel, makeCameras, makeChainRobot, makeUrdfRobot, setupRenderer } from './simWorld';
import { subscribeObjects } from './useSimObjects';

const W = 320;
const H = 240;
const FPS = 10;

export default function SimCameraPublisher() {
  const status = useHardwareStatus();
  const rosbridgeUrl = useSelector((s) => s.ros.rosbridgeUrl);
  const cams = status?.planning?.sim_cameras || [];
  const chain = status?.sim?.chain;
  const key = cams.map((c) => `${c.topic}:${c.view}`).join(',');
  const chainKey = chain ? JSON.stringify(chain.joints.map((j) => j.name)) : '';
  const chainRef = useRef(chain);
  chainRef.current = chain;
  const robotType = useSelector((s) => s.tasks.taskStatus.robotType);
  const [model, setModel] = useState(undefined);
  useEffect(() => {
    let alive = true;
    setModel(undefined);
    loadCustomModel(robotType).then((m) => alive && setModel(m));
    return () => {
      alive = false;
    };
  }, [robotType]);

  useEffect(() => {
    if (!key || model === undefined || (!model && !chainKey)) return undefined;
    const list = key.split(',').map((s) => {
      const i = s.lastIndexOf(':');
      return { topic: s.slice(0, i), view: s.slice(i + 1) };
    });
    let alive = true;
    let timer = null;
    let sub = null;
    let subObjs = null;
    const joints = { names: [], positions: [] };
    let objects = [];
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
    setupRenderer(renderer);
    const w = createWorld();
    let robot;
    try {
      robot = model ? makeUrdfRobot(model) : makeChainRobot(chainRef.current);
    } catch (e) {
      renderer.dispose();
      return undefined;
    }
    w.setRobot(robot.root);
    w.fitTable(robot.length);
    robot.ready.then(() => w.setRobot(robot.root));
    const cams = makeCameras(robot, w, W / H);
    w.helpers.forEach((x) => {
      x.visible = false; // the recorded images show the world only
    });
    const scene = w.scene;
    const sceneCam = cams.scene;
    const wristCam = cams.wrist;

    rosConnectionManager
      .getConnection(rosbridgeUrl)
      .then((ros) => {
        if (!alive) return;
        sub = new ROSLIB.Topic({ ros, name: SIM_TOPIC, messageType: 'sensor_msgs/msg/JointState', throttle_rate: 50 });
        sub.subscribe((m) => {
          joints.names = m.name || [];
          joints.positions = m.position || [];
        });
        subObjs = subscribeObjects(ros, (items) => {
          objects = items;
        });
        const pubs = list.map((c) => ({ ...c, pub: new ROSLIB.Topic({ ros, name: c.topic, messageType: 'sensor_msgs/msg/CompressedImage' }) }));
        // every frame is published (the recorder expects a steady stream); a frame is drawn
        // again only when the joints, the objects or the loaded meshes changed
        let ready = false;
        robot.ready.then(() => {
          ready = true;
        });
        let lastKey = '';
        const cache = {};
        timer = setInterval(() => {
          const k = JSON.stringify([joints.positions, objects, ready]);
          const fresh = k !== lastKey;
          if (fresh) {
            lastKey = k;
            robot.pose(joints.names, joints.positions);
            w.setItems(objects);
          }
          const t = Date.now() / 1000;
          pubs.forEach((c) => {
            if (fresh || !cache[c.topic]) {
              renderer.render(scene, c.view === 'wrist' ? wristCam : sceneCam);
              cache[c.topic] = canvas.toDataURL('image/jpeg', 0.85).split(',')[1];
            }
            const data = cache[c.topic];
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
      if (subObjs) subObjs.unsubscribe();
      w.dispose();
      renderer.dispose();
    };
  }, [key, chainKey, rosbridgeUrl, model]);

  return null;
}
