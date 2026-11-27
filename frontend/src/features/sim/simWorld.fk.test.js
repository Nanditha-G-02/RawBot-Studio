// Forward kinematics of the 3D view's URDF robot (simWorld.makeUrdfRobot) for given joint
// values, written out for comparison with MuJoCo (tests run only when FK_CASES is set).
import fs from 'fs';
import * as THREE from 'three';
import { makeUrdfRobot } from './simWorld';

const cases = process.env.FK_CASES ? JSON.parse(fs.readFileSync(process.env.FK_CASES, 'utf8')) : null;
const maybe = cases ? test : test.skip;

maybe('URDF robot forward kinematics', () => {
  const out = [];
  cases.robots.forEach((rc) => {
    const urdf = fs.readFileSync(rc.urdf, 'utf8');
    const robot = makeUrdfRobot({ urdf, meshMap: {}, id: 'fk', mapping: rc.mapping || [] });
    const res = { name: rc.name, samples: [], reach: robot.length, height: robot.height, tool: robot.toolLink };
    rc.samples.forEach((q) => {
      robot.pose(rc.names, rc.names.map((n) => q[n]));
      robot.root.updateMatrixWorld(true);
      const links = {};
      robot.root.traverse((o) => {
        if (o.userData && o.userData.link) {
          const p = new THREE.Vector3().setFromMatrixPosition(o.matrixWorld);
          links[o.userData.link] = [p.x, p.y, p.z];
        }
      });
      res.samples.push(links);
    });
    out.push(res);
  });
  fs.writeFileSync(process.env.FK_OUT, JSON.stringify(out));
});
