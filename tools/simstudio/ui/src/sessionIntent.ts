import type { JsonObject } from "./api.ts";

export type RuntimeBinding = "physics" | "visual" | "sensors" | "control";
export type RuntimeMode = "headless" | "unreal";

export interface ComposerRobotValues {
  instanceId: string;
  robotRef: string;
  controllerRef: string;
  sensorRigRef: string;
  spawn: { x: number; y: number; z: number; yawDeg: number };
}

export interface ComposerValues {
  sessionId: string;
  mujocoVersion: string;
  seed: number;
  worldRef: string;
  scenarioRef: string;
  robots: ComposerRobotValues[];
  runtimeMode: RuntimeMode;
  requiredBindings: RuntimeBinding[];
}

export interface RobotIntent extends JsonObject {
  instance_id: string;
  package: string;
  controller?: string;
  sensor_rig?: string;
  spawn: {
    position_m: [number, number, number];
    quaternion_wxyz: [number, number, number, number];
  };
}

export interface SessionIntent extends JsonObject {
  schema: "lingtu.sim.session-intent.v1";
  session: {
    session_id: string;
    mujoco_version: string;
    seed: number;
    world: string;
    scenario?: string;
    robots: RobotIntent[];
    runtime: {
      backend: "mujoco";
      mode: RuntimeMode;
      required_bindings: RuntimeBinding[];
    };
  };
}

const SAFE_ID = /^[A-Za-z0-9][A-Za-z0-9_.-]*$/;
const PACKAGE_REF = /^[A-Za-z0-9][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9+_.-]*$/;
const ALLOWED_BINDINGS = new Set<RuntimeBinding>([
  "physics",
  "visual",
  "sensors",
  "control",
]);

function requireText(value: string, label: string): string {
  const normalized = value.trim();
  if (!normalized) {
    throw new Error(`${label} is required`);
  }
  return normalized;
}

function requireId(value: string, label: string): string {
  const normalized = requireText(value, label);
  if (!SAFE_ID.test(normalized)) {
    throw new Error(`${label} must use letters, numbers, dot, underscore, or hyphen`);
  }
  return normalized;
}

function requirePackageRef(value: string, label: string): string {
  const normalized = requireText(value, label);
  if (!PACKAGE_REF.test(normalized)) {
    throw new Error(`${label} must be an exact package reference`);
  }
  return normalized;
}

export function buildSessionIntent(values: ComposerValues): SessionIntent {
  if (!Number.isInteger(values.seed)) {
    throw new Error("seed must be an integer");
  }
  if (
    values.requiredBindings.length === 0 ||
    !values.requiredBindings.includes("physics")
  ) {
    throw new Error("required bindings must include physics");
  }
  if (
    new Set(values.requiredBindings).size !== values.requiredBindings.length ||
    values.requiredBindings.some((binding) => !ALLOWED_BINDINGS.has(binding))
  ) {
    throw new Error("required bindings contain duplicate or unsupported values");
  }

  if (!Array.isArray(values.robots) || values.robots.length === 0) {
    throw new Error("session must contain at least one robot instance");
  }
  if (values.robots.length > 64) {
    throw new Error("session supports at most 64 robot instances");
  }
  const instanceIds = new Set<string>();
  const robots = values.robots.map((value, index): RobotIntent => {
    const instanceId = requireId(value.instanceId, `robots[${index}] instance id`);
    if (instanceIds.has(instanceId)) {
      throw new Error("robot instance ids must be unique");
    }
    instanceIds.add(instanceId);
    const coordinates = [value.spawn.x, value.spawn.y, value.spawn.z] as const;
    if (
      coordinates.some((coordinate) => !Number.isFinite(coordinate))
      || !Number.isFinite(value.spawn.yawDeg)
    ) {
      throw new Error(`robot ${instanceId} spawn values must be finite numbers`);
    }
    const halfYaw = (value.spawn.yawDeg * Math.PI) / 360;
    const clean = (number: number) => Math.abs(number) < 1e-15 ? 0 : number;
    const robot: RobotIntent = {
      instance_id: instanceId,
      package: requirePackageRef(value.robotRef, `robot ${instanceId} package`),
      spawn: {
        position_m: [...coordinates],
        quaternion_wxyz: [clean(Math.cos(halfYaw)), 0, 0, clean(Math.sin(halfYaw))],
      },
    };
    if (value.controllerRef.trim()) {
      robot.controller = requirePackageRef(
        value.controllerRef,
        `robot ${instanceId} controller package`,
      );
    } else if (values.requiredBindings.includes("control")) {
      throw new Error(`robot ${instanceId} requires a controller`);
    }
    if (value.sensorRigRef.trim()) {
      robot.sensor_rig = requirePackageRef(
        value.sensorRigRef,
        `robot ${instanceId} sensor rig package`,
      );
    } else if (values.requiredBindings.includes("sensors")) {
      throw new Error(`robot ${instanceId} requires a sensor rig`);
    }
    return robot;
  });

  const session: SessionIntent["session"] = {
    session_id: requireId(values.sessionId, "session id"),
    mujoco_version: requireText(values.mujocoVersion, "MuJoCo version"),
    seed: values.seed,
    world: requirePackageRef(values.worldRef, "world package"),
    robots,
    runtime: {
      backend: "mujoco",
      mode: values.runtimeMode,
      required_bindings: [...values.requiredBindings],
    },
  };
  if (values.scenarioRef.trim()) {
    session.scenario = requirePackageRef(values.scenarioRef, "scenario package");
  }
  return { schema: "lingtu.sim.session-intent.v1", session };
}

export function resolveWorldRef(
  worldRefs: readonly string[],
  currentWorldRef: string,
  preferredWorldRef?: string | null,
): string {
  if (preferredWorldRef && worldRefs.includes(preferredWorldRef)) return preferredWorldRef;
  if (currentWorldRef && worldRefs.includes(currentWorldRef)) return currentWorldRef;
  return worldRefs[0] ?? "";
}
