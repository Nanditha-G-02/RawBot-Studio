// Copyright 2026 ROBOTIS CO., LTD.
// Licensed under the Apache License, Version 2.0

// The simulated world, shared by the 3D view (SimViewport) and the simulated cameras
// (SimCameraPublisher), so the view shows exactly what is recorded. A studio stage in the
// style of Isaac Sim / Gazebo: charcoal backdrop, a grid floor, a table sized to the robot's
// reach, a key light with soft shadows, PBR materials. One fixed look: frames must not depend
// on the page theme.
//
// Frames: the world group is the robot's base frame (URDF, Z-up, metres); the robot base sits
// on the table top at the origin. Objects come from the scene (the plan episode) and, with
// the MuJoCo backend, their poses from the physics (simbot OBJECTS_TOPIC).

import * as THREE from 'three';
import { buildRobot, parseUrdf, setJoint } from '../robots/UrdfViewer';
import { fetchResolvedUrdf, getRobot, listRobots } from '../robots/robotsApi';

export const STAGE = {
  backdrop: 0x2b2f35,
  floor: 0x353a41,
  gridMajor: 0x5b626c,
  gridMinor: 0x464c55,
  tableTop: 0x8c8f93,
  tableLeg: 0x5d6167,
  robotWhite: 0xeef0f2,
  robotDark: 0x1c1e21,
  camera: 0x6aa6ff,
};
export const TABLE_H = 0.74;

export const robotMaterial = () => new THREE.MeshStandardMaterial({ color: STAGE.robotWhite, metalness: 0.05, roughness: 0.38 });

export function setupRenderer(renderer, { shadows = true } = {}) {
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.NoToneMapping;
  renderer.shadowMap.enabled = shadows;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.setClearColor(STAGE.backdrop);
}

function shadowed(o, cast = true, receive = true) {
  o.traverse((n) => {
    if (n.isMesh) {
      n.castShadow = cast;
      n.receiveShadow = receive;
    }
  });
  return o;
}

function tableGroup() {
  const g = new THREE.Group();
  const top = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 0.04), new THREE.MeshStandardMaterial({ color: STAGE.tableTop, roughness: 0.7 }));
  top.position.z = -0.02;
  top.name = 'top';
  g.add(top);
  const legMat = new THREE.MeshStandardMaterial({ color: STAGE.tableLeg, metalness: 0.3, roughness: 0.6 });
  [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(([sx, sy]) => {
    const leg = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, TABLE_H - 0.04), legMat);
    leg.position.z = -(TABLE_H + 0.04) / 2;
    leg.userData.leg = [sx, sy];
    g.add(leg);
  });
  return shadowed(g);
}

function disposeTree(o) {
  o.traverse((n) => {
    if (n.geometry) n.geometry.dispose();
    if (n.material) (Array.isArray(n.material) ? n.material : [n.material]).forEach((m) => m.dispose());
  });
}

// one scene object: a cube for an object, an open tray for a target (same shapes as the
// MuJoCo world, hardware/mujoco_worker.py), origin at its base centre
function itemMesh(it) {
  const s = it.size || 0.03;
  const color = new THREE.Color(it.color || (it.role === 'target' ? '#2f7fd6' : '#d8532f'));
  const mat = new THREE.MeshStandardMaterial({ color, roughness: 0.5 });
  const g = new THREE.Group();
  if (it.role === 'target') {
    const wall = 0.006;
    const h = Math.min(0.04, s * 0.25);
    const base = new THREE.Mesh(new THREE.BoxGeometry(s, s, wall), mat);
    base.position.z = wall / 2;
    g.add(base);
    [[0, 1], [0, -1], [1, 0], [-1, 0]].forEach(([a, b]) => {
      const w = new THREE.Mesh(new THREE.BoxGeometry(a ? wall : s, b ? wall : s, h), mat);
      w.position.set((a * (s - wall)) / 2, (b * (s - wall)) / 2, h / 2);
      g.add(w);
    });
  } else {
    const cube = new THREE.Mesh(new THREE.BoxGeometry(s, s, s), mat);
    cube.position.z = s / 2;
    g.add(cube);
  }
  g.userData.key = `${it.id}:${it.role}:${s}:${it.color || ''}`;
  return shadowed(g);
}

// the stage. world = robot base frame (Z-up)
export function createWorld() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(STAGE.backdrop);
  scene.fog = new THREE.Fog(STAGE.backdrop, 4, 9);
  const world = new THREE.Group();
  world.rotation.x = -Math.PI / 2;
  scene.add(world);

  const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: STAGE.floor, roughness: 0.95 }));
  floor.position.z = -TABLE_H;
  floor.receiveShadow = true;
  world.add(floor);
  const grid = new THREE.GridHelper(20, 80, STAGE.gridMajor, STAGE.gridMinor);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -TABLE_H + 0.001;
  world.add(grid);

  const table = tableGroup();
  world.add(table);
  const axes = new THREE.AxesHelper(0.12);
  world.add(axes);

  world.add(new THREE.HemisphereLight(0xdfe6ef, 0x30343a, 0.65 * Math.PI));
  const sun = new THREE.DirectionalLight(0xffffff, 0.95 * Math.PI);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  sun.shadow.bias = -0.0003;
  sun.shadow.normalBias = 0.01;
  world.add(sun);
  world.add(sun.target);

  const items = new THREE.Group();
  world.add(items);
  const holder = new THREE.Group();
  world.add(holder);

  let reach = 0.4;
  let tableBox = { cx: 0.2, sx: 1, sy: 0.8 };
  // the table follows the robot: about 2.3 x 2.4 reach, the base near the back edge
  const fitTable = (L) => {
    reach = Math.max(0.15, L || 0.4);
    const sx = Math.max(0.6, reach * 2.3);
    const sy = Math.max(0.6, reach * 2.4);
    const cx = sx / 2 - Math.max(0.12, reach * 0.35);
    tableBox = { cx, sx, sy };
    table.children.forEach((c) => {
      if (c.name === 'top') {
        c.scale.set(sx, sy, 1);
        c.position.x = cx;
      } else if (c.userData.leg) {
        const [a, b] = c.userData.leg;
        c.position.x = cx + a * (sx / 2 - 0.06);
        c.position.y = b * (sy / 2 - 0.06);
      }
    });
    axes.position.set(cx - sx / 2 + 0.06, sy / 2 - 0.06, 0.001);
    axes.scale.setScalar(Math.max(0.6, reach / 0.4));
    const r = Math.max(sx, sy);
    sun.position.set(cx + r * 0.9, -r * 0.8, r * 1.6);
    sun.target.position.set(cx, 0, 0);
    Object.assign(sun.shadow.camera, { left: -r, right: r, top: r, bottom: -r, near: 0.1, far: r * 6 });
    sun.shadow.camera.updateProjectionMatrix();
    scene.fog.near = r * 4;
    scene.fog.far = r * 10;
  };
  fitTable(reach);

  const setRobot = (obj) => {
    while (holder.children.length) holder.remove(holder.children[0]);
    if (obj) holder.add(shadowed(obj));
  };

  // items: [{ id, role, size, color, x, y, yaw }] (planned) or with p: [x, y, z] and
  // q: [w, x, y, z] (physics). Meshes are kept while the set is unchanged; only poses update.
  const meshes = new Map();
  const setItems = (list) => {
    const keep = new Set();
    (list || []).forEach((it) => {
      const key = `${it.id}:${it.role}:${it.size || 0.03}:${it.color || ''}`;
      let m = meshes.get(it.id);
      if (m && m.userData.key !== key) {
        items.remove(m);
        disposeTree(m);
        m = null;
      }
      if (!m) {
        m = itemMesh(it);
        meshes.set(it.id, m);
        items.add(m);
      }
      if (it.p) {
        m.position.set(it.p[0], it.p[1], it.p[2]);
        m.quaternion.set(it.q[1], it.q[2], it.q[3], it.q[0]);
      } else {
        m.position.set(it.x, it.y, 0);
        m.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), ((it.yaw || 0) * Math.PI) / 180);
      }
      keep.add(it.id);
    });
    [...meshes.keys()].forEach((id) => {
      if (!keep.has(id)) {
        const m = meshes.get(id);
        items.remove(m);
        disposeTree(m);
        meshes.delete(id);
      }
    });
  };

  const dispose = () => disposeTree(scene);

  return {
    scene,
    world,
    // helper graphics: shown in the 3D view, never in the simulated camera images
    helpers: [axes],
    setRobot,
    setItems,
    fitTable,
    dispose,
    get reach() {
      return reach;
    },
    get table() {
      return tableBox;
    },
  };
}

// the scene camera looks at the robot and the table in front of it from the front-right and
// above, so the whole workspace and the arm's full height are in the frame
// (called before the camera is added to the Z-up world group: lookAt then works in that frame)
export function placeSceneCamera(cam, robot) {
  const R = Math.max(0.15, robot.length || 0.4); // reach beyond the shoulder
  const H = Math.max(0.05, robot.height || 0.1); // shoulder height above the table
  cam.up.set(0, 0, 1);
  cam.position.set(R * 1.3, -R * 0.85, H + R * 0.8);
  cam.lookAt(R * 0.45, 0, H * 0.3);
}

// where the 3D view starts (orbit target and camera), framing the robot and its workspace
export function viewPose(robot) {
  const R = Math.max(0.15, robot.length || 0.4);
  const H = Math.max(0.05, robot.height || 0.1);
  // from the left side, a little in front: the scene camera (front-right, on its stand) is
  // beyond the workspace, not between the view and the robot
  return { position: [R * 1.0, R * 2.3, H + R * 1.35], target: [R * 0.4, 0, H * 0.5], R };
}

// both cameras in the world's base frame: the scene camera is added to `world`, the wrist
// camera rides the tool link
export function makeCameras(robot, world, aspect = 4 / 3) {
  const sceneCam = new THREE.PerspectiveCamera(55, aspect, 0.01, 30);
  placeSceneCamera(sceneCam, robot);
  world.world.add(sceneCam);
  const wristCam = new THREE.PerspectiveCamera(70, aspect, 0.005, 5);
  if (robot.tool) {
    robot.tool.add(wristCam);
    robot.mountWrist(wristCam);
  }
  return { scene: sceneCam, wrist: wristCam };
}

// ---------- robot models ----------

const UNIT = { deg: Math.PI / 180, mm: 0.001 };
const GRIPPERISH = /grip|finger|jaw|claw/i;
const JAWISH = /finger|jaw|claw/i;

function leafChain(parsed) {
  // the longest moving joint chain from the base
  const kids = {};
  Object.values(parsed.joints).forEach((j) => {
    (kids[j.parent] = kids[j.parent] || []).push(j);
  });
  const moving = (c) => c.filter((x) => x.type !== 'fixed' && !x.mimic).length;
  const walk = (link, depth = 0) => {
    if (depth > 64) return [];
    let best = [];
    (kids[link] || []).forEach((j) => {
      const c = [j, ...walk(j.child, depth + 1)];
      if (moving(c) > moving(best) || (moving(c) === moving(best) && c.length > best.length)) best = c;
    });
    return best;
  };
  return walk(parsed.base);
}

function offsetOf(j) {
  const o = j.el && j.el.querySelector(':scope > origin');
  return ((o && o.getAttribute('xyz')) || '0 0 0').trim().split(/\s+/).map(Number);
}

// reach and shoulder height of a joint chain (metres): the first non-zero offset from the
// base is the shoulder height; the sum of all later offsets is the reach
export function chainReach(chain) {
  const lens = chain.map((j) => Math.hypot(...offsetOf(j)));
  const first = lens.findIndex((l) => l > 1e-6);
  if (first < 0) return { reach: 0.4, height: 0.1 };
  const reach = lens.slice(first + 1).reduce((s, l) => s + l, 0);
  return { reach: reach > 0.05 ? reach : 0.4, height: Math.abs(offsetOf(chain[first])[2]) || 0.1 };
}

// a robot from its URDF. mapping: robot configuration joint rows (urdf_joint, ros_joint,
// direction, offset, units): URDF value = direction * ROS value (in rad / m) + offset
export function makeUrdfRobot({ urdf, meshMap, id, mapping }) {
  const parsed = parseUrdf(urdf);
  const built = buildRobot(parsed, meshMap || {}, id);
  const chain = leafChain(parsed);
  // the wrist camera rides the last arm link - the gripper body, which turns with the wrist -
  // not a jaw, so it does not open and close with the gripper. Trailing fixed joints (tool
  // frames) and jaw joints (named gripper/finger/jaw, or moving a finger/jaw link) are cut; a
  // wrist joint that carries the "gripper_link" body stays.
  let cut = chain.length;
  while (cut > 0 && (chain[cut - 1].type === 'fixed' || GRIPPERISH.test(chain[cut - 1].name) || JAWISH.test(chain[cut - 1].child))) cut -= 1;
  const armChain = chain.slice(0, cut);
  const toolLink = armChain.length ? armChain[armChain.length - 1].child : parsed.base;
  const mimics = Object.values(parsed.joints).filter((j) => j.mimic && built.jointObj[j.name]);
  const byRos = {};
  (mapping || []).forEach((r) => {
    if (r && r.urdf_joint && r.enabled !== false) byRos[r.ros_joint || r.urdf_joint] = r;
  });
  const restyle = () =>
    built.root.traverse((n) => {
      // STL / OBJ carry no colour (the viewer's grey): robot white. DAE keeps its colours.
      if (n.isMesh && n.material && n.material.color && n.material.color.getHex() === 0x8a96a3) {
        n.material.dispose();
        n.material = robotMaterial();
      }
    });
  restyle();
  const ready = Promise.all(built.loads).then(() => {
    restyle();
    shadowed(built.root);
  });
  // reach: the joint offsets after the first one (that one is the shoulder height above the
  // base), the same measure as the backend (robotcaps.chain_reach), so the table here and in
  // MuJoCo are the same size
  const { reach: length, height } = chainReach(chain);

  // which way the tool points, in the tool link's frame: towards the farthest link beyond the
  // tool link (a tool-tip frame, the jaws), else along the last arm link. Computed at the
  // zero pose.
  const descendants = (link, depth = 0) =>
    depth > 32
      ? []
      : Object.values(parsed.joints)
          .filter((j) => j.parent === link)
          .flatMap((j) => [j.child, ...descendants(j.child, depth + 1)]);
  const outward = () => {
    built.root.updateMatrixWorld(true);
    const tool = built.linkObj[toolLink];
    const toolPos = new THREE.Vector3().setFromMatrixPosition(tool.matrixWorld);
    let from = null;
    let to = null;
    let far = 0.01;
    descendants(toolLink).forEach((l) => {
      if (!built.linkObj[l]) return;
      const p = new THREE.Vector3().setFromMatrixPosition(built.linkObj[l].matrixWorld);
      if (p.distanceTo(toolPos) > far) {
        far = p.distanceTo(toolPos);
        to = p;
      }
    });
    if (!to) {
      for (let k = armChain.length - 1; k >= 0 && !from; k -= 1) {
        const p = new THREE.Vector3().setFromMatrixPosition(built.linkObj[armChain[k].parent].matrixWorld);
        if (p.distanceTo(toolPos) > 0.01) from = p;
      }
      to = toolPos.clone();
    } else from = toolPos.clone();
    if (!from) return new THREE.Vector3(0, 0, 1);
    const dWorld = to.sub(from).normalize();
    const inv = new THREE.Quaternion().setFromRotationMatrix(tool.matrixWorld).invert();
    return dWorld.applyQuaternion(inv).normalize();
  };

  const robot = {
    root: built.root,
    ready,
    length,
    height,
    tool: built.linkObj[toolLink],
    toolLink,
    joints: built.jointObj,
    mountWrist(cam) {
      // the simulated gripper (robots/gripper.py): the camera rides its body, beside the finger
      // plane and looking along the fingers, so both fingers show at the sides of the image
      const gb = built.linkObj.sim_gripper_base;
      if (gb) {
        gb.add(cam);
        const off = new THREE.Vector3(0.04, 0, 0.045);
        const up = new THREE.Vector3(1, 0, 0);
        cam.position.copy(off);
        cam.up.copy(up);
        cam.quaternion.setFromRotationMatrix(new THREE.Matrix4().lookAt(off, new THREE.Vector3(0, 0, 0.17), up));
        return;
      }
      const d = outward();
      // beside the flange, looking along the tool
      const side = Math.abs(d.z) < 0.9 ? new THREE.Vector3(0, 0, 1) : new THREE.Vector3(1, 0, 0);
      const off = new THREE.Vector3().crossVectors(d, side).normalize().multiplyScalar(Math.max(0.03, length * 0.08));
      // local frame (lookAt() would take world coordinates): aim at a point on the tool axis
      // ahead of the flange; image up = away from the axis, so the tool shows at the bottom
      const target = d.clone().multiplyScalar(Math.max(0.12, length * 0.35));
      const up = off.clone().normalize();
      cam.position.copy(off);
      cam.up.copy(up);
      cam.quaternion.setFromRotationMatrix(new THREE.Matrix4().lookAt(off, target, up));
    },
    pose(names, positions) {
      const set = {};
      (names || []).forEach((n, i) => {
        const row = byRos[n];
        const jn = row ? row.urdf_joint : n;
        const j = built.jointObj[jn];
        if (!j) return;
        let v = Number(positions[i]) || 0;
        if (row) v = (row.direction === -1 ? -1 : 1) * v * (UNIT[row.units] || 1) + (Number(row.offset) || 0);
        setJoint(j, v);
        set[jn] = v;
      });
      mimics.forEach((m) => {
        if (set[m.mimic.joint] !== undefined) setJoint(built.jointObj[m.name], set[m.mimic.joint] * m.mimic.multiplier + m.mimic.offset);
      });
    },
  };
  return robot;
}

// a robot without a URDF in RawBot Studio (built-in types): the kinematic chain from the
// hardware status, drawn as links and joint housings
export function makeChainRobot(chain) {
  const root = new THREE.Group();
  const moving = {};
  const linkMat = robotMaterial();
  const jointMat = new THREE.MeshStandardMaterial({ color: STAGE.robotDark, metalness: 0.2, roughness: 0.5 });
  let parent = root;
  let length = 0;
  (chain.joints || []).forEach((j) => {
    const origin = new THREE.Group();
    const [x, y, z] = j.xyz || [0, 0, 0];
    origin.position.set(x, y, z);
    const [r, p, w] = j.rpy || [0, 0, 0];
    origin.rotation.set(r, p, w, 'ZYX');
    const len = Math.hypot(x, y, z);
    length += len;
    if (len > 1e-4) {
      const link = new THREE.Mesh(new THREE.BoxGeometry(0.03, len, 0.034), linkMat);
      link.position.set(x / 2, y / 2, z / 2);
      link.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(x, y, z).normalize());
      parent.add(link);
    }
    const motion = new THREE.Group();
    origin.add(motion);
    if (j.type !== 'fixed') {
      const housing = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.044, 24), jointMat);
      const ax = new THREE.Vector3(...(j.axis || [1, 0, 0])).normalize();
      housing.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), ax);
      origin.add(housing);
    }
    parent.add(origin);
    moving[(chain.names || {})[j.name] || j.name] = { j, motion };
    parent = motion;
  });
  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.055, 0.04, 32), linkMat);
  base.rotation.x = Math.PI / 2;
  base.position.z = 0.02;
  root.add(base);
  const tool = new THREE.Group();
  parent.add(tool);
  return {
    root: shadowed(root),
    ready: Promise.resolve(),
    length: length || 0.3,
    height: 0.12,
    tool,
    // chain tools look along their +x (SO-101 family)
    mountWrist(cam) {
      cam.up.set(0, 0, 1);
      cam.rotation.set(0, -Math.PI / 2, -Math.PI / 2);
      cam.position.set(-0.03, 0, 0.03);
    },
    pose(names, positions) {
      (names || []).forEach((n, i) => {
        const m = moving[n];
        if (!m) return;
        const v = Number(positions[i]) || 0;
        const ax = new THREE.Vector3(...(m.j.axis || [1, 0, 0])).normalize();
        if (m.j.type === 'prismatic') m.motion.position.copy(ax.multiplyScalar(v));
        else if (m.j.type !== 'fixed') m.motion.quaternion.setFromAxisAngle(ax, v);
      });
    },
  };
}

// the custom robot (uploaded URDF) behind a server robot type, or null for built-in types
const customCache = {};
export async function loadCustomModel(robotType) {
  if (!robotType) return null;
  if (customCache[robotType]) return customCache[robotType];
  const p = (async () => {
    const { robots = [] } = await listRobots();
    const r = robots.find((x) => x.id.toLowerCase().replace(/[^a-z0-9_]/g, '_') === robotType);
    if (!r) return null;
    const [full, urdf] = await Promise.all([getRobot(r.id), fetchResolvedUrdf(r.id)]);
    return {
      id: r.id,
      name: r.name,
      urdf,
      meshMap: full?.validation?.summary?.mesh_map || {},
      mapping: full?.config?.joints || [],
    };
  })().catch(() => null);
  customCache[robotType] = p;
  const out = await p;
  if (!out) delete customCache[robotType]; // retry later (service may come up)
  return out;
}

export function forgetCustomModel(robotType) {
  delete customCache[robotType];
}
