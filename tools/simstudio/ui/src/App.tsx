import {
  Activity,
  Boxes,
  Globe2,
  Hammer,
  History,
  Orbit,
  Server,
} from "lucide-react";
import { useCallback, useEffect, useMemo, useState } from "react";
import type { ComponentType } from "react";

import {
  API_PREFIX,
  SimStudioClient,
  type CatalogList,
  type HealthStatus,
  type ScenePublication,
  type SimStudioCapabilities,
} from "./api.ts";
import { ImportWorkbench } from "./components/ImportWorkbench.tsx";
import { PackageLibrary } from "./components/PackageLibrary.tsx";
import { ReplayBrowser } from "./components/ReplayBrowser.tsx";
import {
  SessionComposer,
  type ComposedBundle,
} from "./components/SessionComposer.tsx";
import { RunMonitor } from "./components/RunMonitor.tsx";
import { SceneWorkbench } from "./components/SceneWorkbench.tsx";
import { WorldCommand } from "./components/WorldCommand.tsx";
import { errorMessage } from "./format.ts";
import {
  SIMSTUDIO_DEFAULT_VIEW,
  WORLD_COMMAND_NAVIGATION,
  type WorldCommandViewId,
} from "./worldCommandModel.ts";

type ViewId =
  | "world"
  | "packages"
  | "imports"
  | "scene"
  | "composer"
  | "runs"
  | "replay";

interface SecondaryNavigationItem {
  id: ViewId;
  label: string;
  detail: string;
}

const PRIMARY_TARGET: Record<WorldCommandViewId, ViewId> = {
  world: "world",
  models: "packages",
  create: "scene",
  runs: "runs",
  evidence: "replay",
};

const PRIMARY_ICONS: Record<
  WorldCommandViewId,
  ComponentType<{ size?: number; strokeWidth?: number }>
> = {
  world: Globe2,
  models: Boxes,
  create: Hammer,
  runs: Activity,
  evidence: History,
};

const SECONDARY_NAVIGATION: Partial<Record<WorldCommandViewId, SecondaryNavigationItem[]>> = {
  create: [
    { id: "scene", label: "World layout", detail: "场景元素与发布" },
    { id: "imports", label: "Controlled import", detail: "受控资产导入" },
  ],
  runs: [
    { id: "runs", label: "Run monitor", detail: "运行、Readiness 与制品" },
    { id: "composer", label: "Session composer", detail: "World、Robot 与绑定" },
  ],
};

export default function App() {
  const client = useMemo(() => new SimStudioClient(), []);
  const [view, setView] = useState<ViewId>(SIMSTUDIO_DEFAULT_VIEW);
  const [catalog, setCatalog] = useState<CatalogList | null>(null);
  const [catalogLoading, setCatalogLoading] = useState(true);
  const [catalogError, setCatalogError] = useState<string | null>(null);
  const [health, setHealth] = useState<HealthStatus | null>(null);
  const [healthLoading, setHealthLoading] = useState(true);
  const [healthError, setHealthError] = useState<string | null>(null);
  const [capabilities, setCapabilities] = useState<SimStudioCapabilities | null>(null);
  const [capabilitiesError, setCapabilitiesError] = useState<string | null>(null);
  const [pendingBundle, setPendingBundle] = useState<ComposedBundle | null>(null);
  const [preferredWorldRef, setPreferredWorldRef] = useState<string | null>(null);

  const loadCatalog = useCallback(async (): Promise<boolean> => {
    try {
      const result = await client.listPackages();
      setCatalogError(null);
      setCatalog(result);
      return true;
    } catch (reason) {
      setCatalogError(errorMessage(reason));
      return false;
    } finally {
      setCatalogLoading(false);
    }
  }, [client]);

  const loadHealth = useCallback(async () => {
    setHealthLoading(true);
    try {
      const result = await client.health();
      setHealthError(null);
      setHealth(result);
    } catch (reason) {
      setHealth(null);
      setHealthError(errorMessage(reason));
    } finally {
      setHealthLoading(false);
    }
  }, [client]);

  const loadCapabilities = useCallback(async () => {
    try {
      const result = await client.capabilities();
      setCapabilitiesError(null);
      setCapabilities(result);
    } catch (reason) {
      setCapabilities(null);
      setCapabilitiesError(errorMessage(reason));
    }
  }, [client]);

  const reloadCatalog = useCallback(() => {
    setCatalogLoading(true);
    setCatalogError(null);
    void loadCatalog();
  }, [loadCatalog]);

  const refreshOverview = useCallback(() => {
    setCatalogLoading(true);
    setCatalogError(null);
    setCapabilitiesError(null);
    void Promise.all([loadCatalog(), loadHealth(), loadCapabilities()]);
  }, [loadCapabilities, loadCatalog, loadHealth]);

  const usePublishedWorldForNewSession = useCallback(async (publication: ScenePublication) => {
    setCatalogLoading(true);
    setCatalogError(null);
    try {
      if (!await loadCatalog()) return;
      setPreferredWorldRef(publication.publication.package.ref);
      setView("composer");
    } catch (reason) {
      setCatalogError(errorMessage(reason));
    } finally {
      setCatalogLoading(false);
    }
  }, [loadCatalog]);

  useEffect(() => {
    const initialLoad = window.setTimeout(() => {
      void Promise.all([loadCatalog(), loadHealth(), loadCapabilities()]);
    }, 0);
    return () => window.clearTimeout(initialLoad);
  }, [loadCapabilities, loadCatalog, loadHealth]);

  const primaryView = primaryViewFor(view);
  const secondaryNavigation = SECONDARY_NAVIGATION[primaryView] ?? [];
  const serviceTone = healthLoading
    ? "pending"
    : healthError || !health || health.status !== "ok"
      ? "failed"
      : "ready";
  const serviceLabel = healthLoading
    ? "Checking"
    : healthError || !health
      ? "Unavailable"
      : health.status.toUpperCase();
  const serviceTitle = [healthError, capabilitiesError].filter(Boolean).join(" · ")
    || "本地 SimStudio 服务状态";

  function openPrimary(nextView: WorldCommandViewId) {
    setView(PRIMARY_TARGET[nextView]);
  }

  return (
    <div
      className={view === "world" ? "lt-shell lt-shell--forge" : "lt-shell"}
      data-lt-theme="lightfield"
    >
      <a className="lt-skip-link" href="#main-content">跳到主要内容</a>
      {view !== "world" ? <header className="lt-appbar">
        <button className="lt-brand" type="button" onClick={() => setView(SIMSTUDIO_DEFAULT_VIEW)}>
          <span className="lt-brand__mark" aria-hidden="true">
            <Orbit size={21} strokeWidth={1.55} />
          </span>
          <span>
            <strong>LingTu</strong>
            <small>SimStudio</small>
          </span>
        </button>

        <nav className="lt-primary-nav" aria-label="SimStudio 主要功能">
          {WORLD_COMMAND_NAVIGATION.map((item) => {
            const Icon = PRIMARY_ICONS[item.id];
            const active = primaryView === item.id;
            return (
              <button
                type="button"
                className={active ? "lt-primary-nav__item is-active" : "lt-primary-nav__item"}
                aria-current={active ? "page" : undefined}
                title={item.detail}
                onClick={() => openPrimary(item.id)}
                key={item.id}
              >
                <Icon aria-hidden="true" size={17} strokeWidth={1.65} />
                <span>{item.label}</span>
              </button>
            );
          })}
        </nav>

        <div className="lt-appbar__truth" title={serviceTitle}>
          <span className="lt-service-state" data-tone={serviceTone}>
            <span className="lt-service-state__mark" aria-hidden="true" />
            <Server aria-hidden="true" size={15} strokeWidth={1.7} />
            <span>API</span>
            <strong>{serviceLabel}</strong>
          </span>
          <span className="lt-runtime-state">
            Run service {health?.runtime_bound ? "bound" : "not bound"}
          </span>
          <button className="lt-appbar__action" type="button" onClick={() => setView("composer")}>
            Compose session
          </button>
        </div>
      </header> : null}

      <main
        id="main-content"
        className={view === "world" ? "lt-main lt-main--world" : "lt-main lt-main--workbench"}
        tabIndex={-1}
      >
        {view === "world" ? (
          <WorldCommand
            health={health}
            healthLoading={healthLoading}
            healthError={healthError}
            capabilities={capabilities}
            capabilitiesError={capabilitiesError}
            catalog={catalog}
            catalogLoading={catalogLoading}
            catalogError={catalogError}
            pendingBundle={pendingBundle}
            onRefresh={refreshOverview}
            onNavigate={openPrimary}
            onOpenComposer={() => setView("composer")}
          />
        ) : (
          <div className="lt-workspace">
            {secondaryNavigation.length > 0 ? (
              <nav className="lt-context-nav" aria-label={`${primaryView} 工作区`}>
                {secondaryNavigation.map((item) => (
                  <button
                    type="button"
                    className={view === item.id ? "is-active" : ""}
                    aria-current={view === item.id ? "page" : undefined}
                    onClick={() => setView(item.id)}
                    key={item.id}
                  >
                    <strong>{item.label}</strong>
                    <span>{item.detail}</span>
                  </button>
                ))}
              </nav>
            ) : null}

            {view === "scene" ? (
              <div className="lt-mobile-authoring-note" role="note">
                <Globe2 aria-hidden="true" size={20} strokeWidth={1.65} />
                <span>
                  <strong>Geometry authoring is desktop only</strong>
                  <small>移动端保留监控与证据查看；请在 768px 以上视口编辑世界几何。</small>
                </span>
              </div>
            ) : null}

            <div
              className={`legacy-workbench legacy-workbench--${view}`}
              data-lt-surface="workbench"
            >
              {view === "packages" ? (
                <PackageLibrary
                  client={client}
                  catalog={catalog}
                  loading={catalogLoading}
                  error={catalogError}
                  onReload={reloadCatalog}
                />
              ) : null}
              {view === "imports" ? (
                <ImportWorkbench client={client} onCatalogChanged={reloadCatalog} />
              ) : null}
              {view === "scene" ? (
                <SceneWorkbench
                  client={client}
                  onUsePublishedWorldForNewSession={usePublishedWorldForNewSession}
                />
              ) : null}
              {view === "composer" ? (
                <SessionComposer
                  client={client}
                  catalog={catalog}
                  catalogLoading={catalogLoading}
                  catalogError={catalogError}
                  onReloadCatalog={reloadCatalog}
                  preferredWorldRef={preferredWorldRef}
                  onBundleComposed={setPendingBundle}
                  onOpenRunMonitor={() => setView("runs")}
                />
              ) : null}
              {view === "runs" ? (
                <RunMonitor
                  client={client}
                  pendingBundle={pendingBundle}
                  maxPreviewBytes={capabilities?.artifact_preview.max_bytes}
                />
              ) : null}
              {view === "replay" ? <ReplayBrowser client={client} /> : null}
            </div>
          </div>
        )}
      </main>

      {view !== "world" ? <footer className="lt-contract-footer">
        <span>{API_PREFIX}</span>
        <span>Field-isolated local simulation</span>
      </footer> : null}
    </div>
  );
}

function primaryViewFor(view: ViewId): WorldCommandViewId {
  if (view === "packages") return "models";
  if (view === "imports" || view === "scene") return "create";
  if (view === "composer" || view === "runs") return "runs";
  if (view === "replay") return "evidence";
  return "world";
}
