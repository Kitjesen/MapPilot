import {
  ArrowRight,
  Bot,
  CheckCircle2,
  Cpu,
  Layers3,
  LoaderCircle,
  Map,
  Plus,
  Radio,
  Trash2,
} from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { FormEvent } from "react";

import type {
  CatalogList,
  LaunchProfile,
  PackageDetail,
  PackageSummary,
  SimStudioClient,
  StudioRecord,
} from "../api.ts";
import { errorMessage, operationKey, stringField } from "../format.ts";
import {
  buildSessionIntent,
  type ComposerRobotValues,
  type ComposerValues,
  type RuntimeBinding,
  resolveWorldRef,
} from "../sessionIntent.ts";
import { ErrorPanel, SectionHeading, StatusBadge } from "./Common.tsx";

export interface ComposedBundle {
  record: StudioRecord;
  launchProfile: LaunchProfile;
}

interface SessionComposerProps {
  client: SimStudioClient;
  catalog: CatalogList | null;
  catalogLoading: boolean;
  catalogError: string | null;
  onReloadCatalog: () => void;
  preferredWorldRef?: string | null;
  onBundleComposed: (bundle: ComposedBundle) => void;
  onOpenRunMonitor: () => void;
}

interface RobotDraft extends ComposerRobotValues {
  editorId: string;
  detail: PackageDetail | null;
  loading: boolean;
}

function robotDraft(editorId: string, instanceId: string, x: number): RobotDraft {
  return {
    editorId,
    instanceId,
    robotRef: "",
    controllerRef: "",
    sensorRigRef: "",
    spawn: { x, y: 0, z: 0, yawDeg: 0 },
    detail: null,
    loading: false,
  };
}

function packagesOfKind(packages: PackageSummary[], kind: string) {
  return packages.filter((item) => item.package.kind === kind);
}

function packageOption(item: PackageSummary) {
  return (
    <option value={item.package.ref} key={item.package.ref}>
      {item.package.ref}
    </option>
  );
}

export function SessionComposer({
  client,
  catalog,
  catalogLoading,
  catalogError,
  onReloadCatalog,
  preferredWorldRef,
  onBundleComposed,
  onOpenRunMonitor,
}: SessionComposerProps) {
  const packages = useMemo(() => catalog?.packages ?? [], [catalog]);
  const worlds = useMemo(() => packagesOfKind(packages, "world"), [packages]);
  const robots = useMemo(() => packagesOfKind(packages, "robot"), [packages]);
  const controllers = useMemo(() => packagesOfKind(packages, "controller"), [packages]);
  const sensorRigs = useMemo(() => packagesOfKind(packages, "sensor_rig"), [packages]);
  const scenarios = useMemo(() => packagesOfKind(packages, "scenario"), [packages]);

  const [sessionId, setSessionId] = useState("");
  const [mujocoVersion, setMujocoVersion] = useState("3.10.0");
  const [seed, setSeed] = useState(1);
  const [worldRef, setWorldRef] = useState("");
  const [scenarioRef, setScenarioRef] = useState("");
  const nextRobotEditorId = useRef(2);
  const [robotInstances, setRobotInstances] = useState<RobotDraft[]>([
    robotDraft("robot-editor-1", "robot_01", 0),
  ]);
  const [launchProfile, setLaunchProfile] = useState<LaunchProfile>("visual");
  const [bindings, setBindings] = useState<RuntimeBinding[]>([
    "physics",
    "visual",
    "sensors",
    "control",
  ]);
  const [formError, setFormError] = useState<string | null>(null);
  const [submitting, setSubmitting] = useState(false);
  const [bundle, setBundle] = useState<StudioRecord | null>(null);
  const [dismissedPreferredWorldRef, setDismissedPreferredWorldRef] = useState<string | null>(null);

  const worldRefs = useMemo(() => worlds.map((item) => item.package.ref), [worlds]);

  const effectiveWorldRef = resolveWorldRef(
    worldRefs,
    worldRef,
    preferredWorldRef !== dismissedPreferredWorldRef ? preferredWorldRef : null,
  );
  const selectRobotPackage = useCallback(async (editorId: string, reference: string) => {
    setRobotInstances((current) => current.map((robot) => robot.editorId === editorId
      ? {
          ...robot,
          robotRef: reference,
          controllerRef: "",
          sensorRigRef: "",
          detail: null,
          loading: true,
        }
      : robot));
    try {
      const detail = await client.inspectPackage("robot", reference);
      const defaults = detail.manifest_spec.defaults;
      const values = defaults && typeof defaults === "object" && !Array.isArray(defaults)
        ? defaults as Record<string, unknown>
        : {};
      setRobotInstances((current) => current.map((robot) =>
        robot.editorId === editorId && robot.robotRef === reference
          ? {
              ...robot,
              controllerRef: typeof values.controller === "string" ? values.controller : "",
              sensorRigRef: typeof values.sensor_rig === "string" ? values.sensor_rig : "",
              detail,
              loading: false,
            }
          : robot));
    } catch (reason: unknown) {
      setRobotInstances((current) => current.map((robot) =>
        robot.editorId === editorId && robot.robotRef === reference
          ? { ...robot, loading: false }
          : robot));
      setFormError(`无法读取机器人默认绑定: ${errorMessage(reason)}`);
    }
  }, [client]);

  useEffect(() => {
    const firstReference = robots[0]?.package.ref;
    if (!firstReference) return undefined;
    const pending = robotInstances
      .filter((robot) => !robot.robotRef && !robot.loading)
      .map((robot) => robot.editorId);
    if (pending.length === 0) return undefined;
    const timer = window.setTimeout(() => {
      for (const editorId of pending) {
        void selectRobotPackage(editorId, firstReference);
      }
    }, 0);
    return () => window.clearTimeout(timer);
  }, [robotInstances, robots, selectRobotPackage]);

  function updateRobot(editorId: string, update: Partial<RobotDraft>) {
    setRobotInstances((current) => current.map((robot) =>
      robot.editorId === editorId ? { ...robot, ...update } : robot));
  }

  function addRobot() {
    if (robotInstances.length >= 64) return;
    const ordinal = robotInstances.length + 1;
    const editorId = `robot-editor-${nextRobotEditorId.current}`;
    nextRobotEditorId.current += 1;
    const created = robotDraft(editorId, `robot_${String(ordinal).padStart(2, "0")}`, (ordinal - 1) * 2);
    setRobotInstances((current) => [...current, created]);
  }

  function removeRobot(editorId: string) {
    if (robotInstances.length <= 1) return;
    setRobotInstances((current) => current.filter((robot) => robot.editorId !== editorId));
  }

  function changeProfile(profile: LaunchProfile) {
    setLaunchProfile(profile);
    setBindings((current) => {
      const next = new Set(current);
      next.add("physics");
      if (profile === "visual") next.add("visual");
      else next.delete("visual");
      return ["physics", "visual", "sensors", "control"].filter((item) =>
        next.has(item as RuntimeBinding),
      ) as RuntimeBinding[];
    });
  }

  function toggleBinding(binding: RuntimeBinding, checked: boolean) {
    if (binding === "physics" || (binding === "visual" && launchProfile === "visual")) return;
    setBindings((current) => {
      const next = new Set(current);
      if (checked) next.add(binding);
      else next.delete(binding);
      return ["physics", "visual", "sensors", "control"].filter((item) =>
        next.has(item as RuntimeBinding),
      ) as RuntimeBinding[];
    });
  }

  async function submit(event: FormEvent<HTMLFormElement>) {
    event.preventDefault();
    setFormError(null);
    setBundle(null);
    if (bindings.includes("sensors") && robotInstances.some((robot) => !robot.sensorRigRef)) {
      setFormError("要求 sensors 绑定时，每台机器人都必须选择 Sensor Rig。");
      return;
    }
    if (bindings.includes("control") && robotInstances.some((robot) => !robot.controllerRef)) {
      setFormError("要求 control 绑定时，每台机器人都必须选择 Controller。");
      return;
    }

    const values: ComposerValues = {
      sessionId,
      mujocoVersion,
      seed,
      worldRef: effectiveWorldRef,
      scenarioRef,
      robots: robotInstances.map((robot) => ({
        instanceId: robot.instanceId,
        robotRef: robot.robotRef,
        controllerRef: robot.controllerRef,
        sensorRigRef: robot.sensorRigRef,
        spawn: robot.spawn,
      })),
      runtimeMode: launchProfile === "visual" ? "unreal" : "headless",
      requiredBindings: bindings,
    };
    try {
      setSubmitting(true);
      const intent = buildSessionIntent(values);
      const draft = await client.createDraft(intent, operationKey("draft"));
      const composed = await client.composeDraft(
        draft.id,
        draft.revision,
        operationKey("compose"),
      );
      setBundle(composed);
      onBundleComposed({ record: composed, launchProfile });
    } catch (reason) {
      setFormError(errorMessage(reason));
    } finally {
      setSubmitting(false);
    }
  }

  const unavailable = !catalogLoading && (worlds.length === 0 || robots.length === 0);

  return (
    <section aria-labelledby="composer-title">
      <SectionHeading
        id="composer-title"
        title="Session Composer"
        description="从真实 Catalog 选择内容，生成确定性的 SessionIntent、Draft 和 Bundle。"
      />

      {catalogError ? (
        <ErrorPanel title="无法加载 Composer 选项" message={catalogError} onRetry={onReloadCatalog} />
      ) : null}
      {unavailable ? (
        <ErrorPanel
          title="Catalog 不满足编排条件"
          message="至少需要一个 world 和一个 robot Package。"
          onRetry={onReloadCatalog}
        />
      ) : null}

      <form className="composer-layout" onSubmit={submit}>
        <div className="composer-form">
          <fieldset className="form-section" disabled={catalogLoading || submitting || unavailable}>
            <legend>
              <Layers3 aria-hidden="true" size={18} strokeWidth={1.7} />
              会话身份
            </legend>
            <div className="form-grid form-grid--three">
              <label className="field">
                <span>Session ID</span>
                <input
                  required
                  value={sessionId}
                  onChange={(event) => setSessionId(event.target.value)}
                  placeholder="factory_validation"
                  pattern="[A-Za-z0-9][A-Za-z0-9_.-]*"
                />
                <small>稳定标识，只允许字母、数字、点、下划线和连字符。</small>
              </label>
              <label className="field">
                <span>MuJoCo 版本</span>
                <input
                  required
                  value={mujocoVersion}
                  onChange={(event) => setMujocoVersion(event.target.value)}
                />
              </label>
              <label className="field">
                <span>随机种子</span>
                <input
                  required
                  type="number"
                  step="1"
                  value={seed}
                  onChange={(event) => setSeed(event.target.valueAsNumber)}
                />
              </label>
            </div>
          </fieldset>

          <fieldset className="form-section" disabled={catalogLoading || submitting || unavailable}>
            <legend>
              <Map aria-hidden="true" size={18} strokeWidth={1.7} />
              世界与场景
            </legend>
            <div className="form-grid form-grid--two">
              <label className="field">
                <span>World Package</span>
                <select
                  required
                  value={effectiveWorldRef}
                  onChange={(event) => {
                    setDismissedPreferredWorldRef(preferredWorldRef ?? null);
                    setWorldRef(event.target.value);
                  }}
                >
                  {worlds.map(packageOption)}
                </select>
              </label>
              <label className="field">
                <span>Scenario Package</span>
                <select value={scenarioRef} onChange={(event) => setScenarioRef(event.target.value)}>
                  <option value="">不绑定 Scenario</option>
                  {scenarios.map(packageOption)}
                </select>
              </label>
            </div>
          </fieldset>

          <fieldset className="form-section" disabled={catalogLoading || submitting || unavailable}>
            <legend>
              <Bot aria-hidden="true" size={18} strokeWidth={1.7} />
              机器人实例
            </legend>
            <div className="robot-instance-toolbar">
              <p>所有实例进入同一个会话级 MuJoCo 模型，并由同一份 VisualPlan 驱动 UE。</p>
              <button
                className="button button--quiet"
                type="button"
                disabled={robotInstances.length >= 64}
                onClick={addRobot}
              >
                <Plus aria-hidden="true" size={16} strokeWidth={1.8} />
                添加机器人
              </button>
            </div>
            <div className="robot-instance-list">
              {robotInstances.map((robot, index) => (
                <article className="robot-instance-card" key={robot.editorId}>
                  <header>
                    <div>
                      <span>Instance {String(index + 1).padStart(2, "0")}</span>
                      <strong>{robot.instanceId || "未命名机器人"}</strong>
                    </div>
                    <div>
                      {robot.loading ? <StatusBadge value="loading" /> : null}
                      <button
                        className="icon-button icon-button--danger"
                        type="button"
                        aria-label={`删除 ${robot.instanceId || `机器人 ${index + 1}`}`}
                        disabled={robotInstances.length === 1}
                        onClick={() => removeRobot(robot.editorId)}
                      >
                        <Trash2 aria-hidden="true" size={16} strokeWidth={1.7} />
                      </button>
                    </div>
                  </header>
                  <div className="form-grid form-grid--two">
                    <label className="field">
                      <span>Robot Package</span>
                      <select
                        required
                        value={robot.robotRef}
                        onChange={(event) => void selectRobotPackage(robot.editorId, event.target.value)}
                      >
                        {robots.map(packageOption)}
                      </select>
                    </label>
                    <label className="field">
                      <span>Instance ID</span>
                      <input
                        required
                        value={robot.instanceId}
                        onChange={(event) => updateRobot(robot.editorId, { instanceId: event.target.value })}
                        pattern="[A-Za-z0-9][A-Za-z0-9_.-]*"
                      />
                    </label>
                    <label className="field">
                      <span>Controller</span>
                      <select
                        value={robot.controllerRef}
                        onChange={(event) => updateRobot(robot.editorId, { controllerRef: event.target.value })}
                      >
                        <option value="">不绑定 Controller</option>
                        {controllers.map(packageOption)}
                      </select>
                    </label>
                    <label className="field">
                      <span>Sensor Rig</span>
                      <select
                        value={robot.sensorRigRef}
                        onChange={(event) => updateRobot(robot.editorId, { sensorRigRef: event.target.value })}
                      >
                        <option value="">不绑定 Sensor Rig</option>
                        {sensorRigs.map(packageOption)}
                      </select>
                    </label>
                  </div>
                  <div className="spawn-editor" aria-label={`${robot.instanceId} 出生位姿`}>
                    <span>Spawn pose</span>
                    {(["x", "y", "z", "yawDeg"] as const).map((axis) => (
                      <label key={axis}>
                        <span>{axis === "yawDeg" ? "YAW°" : `${axis.toUpperCase()} m`}</span>
                        <input
                          type="number"
                          step={axis === "yawDeg" ? "1" : "0.01"}
                          value={robot.spawn[axis]}
                          onChange={(event) => updateRobot(robot.editorId, {
                            spawn: { ...robot.spawn, [axis]: event.target.valueAsNumber },
                          })}
                        />
                      </label>
                    ))}
                  </div>
                </article>
              ))}
            </div>
          </fieldset>

          <fieldset className="form-section" disabled={catalogLoading || submitting || unavailable}>
            <legend>
              <Cpu aria-hidden="true" size={18} strokeWidth={1.7} />
              Runtime
            </legend>
            <div className="segmented-control" aria-label="运行模式">
              {(["visual", "headless"] as const).map((profile) => (
                <label key={profile}>
                  <input
                    type="radio"
                    name="launch-profile"
                    value={profile}
                    checked={launchProfile === profile}
                    onChange={() => changeProfile(profile)}
                  />
                  <span>{profile === "visual" ? "Visual / Unreal" : "Headless"}</span>
                </label>
              ))}
            </div>
            <div className="binding-grid">
              {(["physics", "visual", "sensors", "control"] as const).map((binding) => {
                const locked = binding === "physics" || (binding === "visual" && launchProfile === "visual");
                return (
                  <label key={binding}>
                    <input
                      type="checkbox"
                      checked={bindings.includes(binding)}
                      disabled={locked || (binding === "visual" && launchProfile === "headless")}
                      onChange={(event) => toggleBinding(binding, event.target.checked)}
                    />
                    <span>{binding}</span>
                  </label>
                );
              })}
            </div>
          </fieldset>

          {formError ? <ErrorPanel title="会话编译失败" message={formError} /> : null}

          <div className="form-actions">
            <button
              className="button button--primary"
              type="submit"
              disabled={catalogLoading || submitting || unavailable}
            >
              {submitting ? (
                <LoaderCircle className="spin" aria-hidden="true" size={17} strokeWidth={1.8} />
              ) : (
                <Radio aria-hidden="true" size={17} strokeWidth={1.8} />
              )}
              {submitting ? "正在编译" : "创建并编译 Bundle"}
            </button>
          </div>
        </div>

        <aside className="bundle-panel" aria-live="polite">
          <h2>Compiled Bundle</h2>
          {!bundle ? (
            <div className="bundle-placeholder">
              <Layers3 aria-hidden="true" size={28} strokeWidth={1.4} />
              <p>完成编译后，这里显示后端返回的 Bundle 身份和会话摘要。</p>
            </div>
          ) : (
            <div className="bundle-result">
              <CheckCircle2 aria-hidden="true" size={30} strokeWidth={1.5} />
              <StatusBadge value={bundle.status} />
              <dl>
                <div>
                  <dt>Bundle ID</dt>
                  <dd><code>{bundle.id}</code></dd>
                </div>
                <div>
                  <dt>Session</dt>
                  <dd>{stringField(bundle.payload.session_id)}</dd>
                </div>
                <div>
                  <dt>Launch Profile</dt>
                  <dd>{launchProfile}</dd>
                </div>
              </dl>
              <button className="button button--primary" type="button" onClick={onOpenRunMonitor}>
                前往 Run Monitor
                <ArrowRight aria-hidden="true" size={17} strokeWidth={1.8} />
              </button>
            </div>
          )}
        </aside>
      </form>
    </section>
  );
}
