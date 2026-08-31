import assert from "node:assert/strict";
import test from "node:test";

import { buildSessionIntent } from "../src/sessionIntent.ts";

test("the composer emits a canonical multi-robot SessionIntent", () => {
  const intent = buildSessionIntent({
    sessionId: "factory_validation",
    mujocoVersion: "3.10.0",
    seed: 20260809,
    worldRef: "factory_park_hf@1.0.0",
    scenarioRef: "",
    robots: [
      {
        instanceId: "thunder_01",
        robotRef: "thunderv4@1.0.3",
        controllerRef: "thunderv4_locomotion@1.0.0",
        sensorRigRef: "thunderv4_navigation@1.0.0",
        spawn: { x: 0, y: -76, z: 0, yawDeg: 90 },
      },
      {
        instanceId: "cart_01",
        robotRef: "omni_cart@1.0.0",
        controllerRef: "omni_cart_drive@1.0.0",
        sensorRigRef: "omni_cart_navigation@1.0.0",
        spawn: { x: 3, y: -76, z: 0, yawDeg: 0 },
      },
    ],
    runtimeMode: "unreal",
    requiredBindings: ["physics", "visual", "sensors", "control"],
  });

  assert.deepEqual(intent, {
    schema: "lingtu.sim.session-intent.v1",
    session: {
      session_id: "factory_validation",
      mujoco_version: "3.10.0",
      seed: 20260809,
      world: "factory_park_hf@1.0.0",
      robots: [
        {
          instance_id: "thunder_01",
          package: "thunderv4@1.0.3",
          controller: "thunderv4_locomotion@1.0.0",
          sensor_rig: "thunderv4_navigation@1.0.0",
          spawn: {
            position_m: [0, -76, 0],
            quaternion_wxyz: [Math.cos(Math.PI / 4), 0, 0, Math.sin(Math.PI / 4)],
          },
        },
        {
          instance_id: "cart_01",
          package: "omni_cart@1.0.0",
          controller: "omni_cart_drive@1.0.0",
          sensor_rig: "omni_cart_navigation@1.0.0",
          spawn: {
            position_m: [3, -76, 0],
            quaternion_wxyz: [1, 0, 0, 0],
          },
        },
      ],
      runtime: {
        backend: "mujoco",
        mode: "unreal",
        required_bindings: ["physics", "visual", "sensors", "control"],
      },
    },
  });
});

test("optional bindings remain optional and robot instance ids are unique", () => {
  const minimal = buildSessionIntent(minimalInput());
  const robot = minimal.session.robots[0];
  assert.equal("controller" in robot, false);
  assert.equal("sensor_rig" in robot, false);
  assert.throws(
    () => buildSessionIntent({ ...minimalInput(), requiredBindings: ["visual"] }),
    /physics/,
  );
  assert.throws(
    () => buildSessionIntent({
      ...minimalInput(),
      robots: [minimalRobot(), minimalRobot()],
    }),
    /unique/,
  );
  assert.throws(
    () => buildSessionIntent({
      ...minimalInput(),
      requiredBindings: ["physics", "sensors"],
    }),
    /sensor rig/,
  );
});

function minimalRobot() {
  return {
    instanceId: "robot_01",
    robotRef: "omni_cart@1.0.0",
    controllerRef: "",
    sensorRigRef: "",
    spawn: { x: 0, y: 0, z: 0, yawDeg: 0 },
  };
}

function minimalInput() {
  return {
    sessionId: "safe_session",
    mujocoVersion: "3.10.0",
    seed: 1,
    worldRef: "open_field@1.0.0",
    scenarioRef: "",
    robots: [minimalRobot()],
    runtimeMode: "headless" as const,
    requiredBindings: ["physics" as const],
  };
}
