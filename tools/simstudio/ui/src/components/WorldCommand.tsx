import {
  AlertTriangle,
  Bot,
  Box,
  ChevronDown,
  CirclePlay,
  CloudUpload,
  Grid2X2,
  Layers3,
  MapPinned,
  Mountain,
  MousePointer2,
  PersonStanding,
  RefreshCw,
  Save,
  Sparkles,
  SquareDashed,
  X,
} from "lucide-react";
import { useState } from "react";
import type { ComponentType } from "react";

import type {
  CatalogList,
  HealthStatus,
  SimStudioCapabilities,
} from "../api.ts";
import {
  buildWorldCommandState,
  createWorldForgeEditorState,
  selectWorldForgeAsset,
  updateWorldForgeRoadInspector,
  WORLD_FORGE_ASSETS,
  type WorldCommandViewId,
  type WorldForgeToolId,
} from "../worldCommandModel.ts";
import type { ComposedBundle } from "./SessionComposer.tsx";

interface WorldCommandProps {
  health: HealthStatus | null;
  healthLoading: boolean;
  healthError: string | null;
  capabilities: SimStudioCapabilities | null;
  capabilitiesError: string | null;
  catalog: CatalogList | null;
  catalogLoading: boolean;
  catalogError: string | null;
  pendingBundle: ComposedBundle | null;
  onRefresh: () => void;
  onNavigate: (view: WorldCommandViewId) => void;
  onOpenComposer: () => void;
}

interface ToolItem {
  id: WorldForgeToolId;
  label: string;
  icon: ComponentType<{ size?: number; strokeWidth?: number }>;
}

const TOOLS: readonly ToolItem[] = [
  { id: "select", label: "Select", icon: SquareDashed },
  { id: "terrain", label: "Terrain", icon: Mountain },
  { id: "objects", label: "Objects", icon: Box },
  { id: "agents", label: "Agents", icon: PersonStanding },
  { id: "layers", label: "Layers", icon: Grid2X2 },
];

const PROMPT_DEFAULT = "Create a realistic factory park with roads, warehouses, tanks and patrol routes";

export function WorldCommand({
  health,
  healthLoading,
  healthError,
  capabilities,
  capabilitiesError,
  catalog,
  catalogLoading,
  catalogError,
  pendingBundle,
  onRefresh,
  onNavigate,
  onOpenComposer,
}: WorldCommandProps) {
  const [editor, setEditor] = useState(createWorldForgeEditorState);
  const [prompt, setPrompt] = useState(PROMPT_DEFAULT);
  const [inspectorOpen, setInspectorOpen] = useState(true);
  const [worldMenuOpen, setWorldMenuOpen] = useState(false);
  const [notice, setNotice] = useState<string | null>(null);
  const [messageDismissed, setMessageDismissed] = useState(false);
  const command = buildWorldCommandState({ health, healthLoading, healthError });
  const serviceReady = health?.status === "ok";
  const packageCount = catalog?.packages.length ?? 0;
  const warningText = worldWarning({
    healthLoading,
    healthError,
    catalogLoading,
    catalogError,
    capabilities,
    capabilitiesError,
  });

  function generateFromPrompt() {
    if (!prompt.trim()) return;
    setMessageDismissed(false);
    setNotice("Prompt staged · open Create to validate and publish this world change");
  }

  function openRunFlow() {
    if (pendingBundle?.launchProfile === "visual") {
      onNavigate("runs");
      return;
    }
    onOpenComposer();
  }

  return (
    <section className="world-forge" aria-label="World Forge authoring preview">
      <header className="world-forge__topbar">
        <button className="world-forge__brand" type="button" aria-label="World Forge home">
          World Forge
        </button>

        <div className="world-forge__project">
          <button
            className="world-forge__project-select"
            type="button"
            aria-expanded={worldMenuOpen}
            onClick={() => setWorldMenuOpen((open) => !open)}
          >
            Factory Park
            <ChevronDown aria-hidden="true" size={15} strokeWidth={1.8} />
          </button>
          <span className="world-forge__divider" aria-hidden="true" />
          <span className="world-forge__presence" title="Collaboration data is illustrative in this authoring preview">
            <span className="world-forge__presence-dot" aria-hidden="true" />
            3 demo
          </span>
          <span className="world-forge__avatars" aria-label="Demo collaborators">
            <span data-avatar="alex">A</span>
            <span data-avatar="priya">P</span>
            <span data-avatar="jordan">J</span>
          </span>
          {worldMenuOpen ? (
            <div className="world-forge__project-menu" role="dialog" aria-label="Selected world">
              <strong>Factory Park HF</strong>
              <span>220 × 180 m · authoring preview</span>
              <small>{packageCount} catalog packages available</small>
            </div>
          ) : null}
        </div>

        <div className="world-forge__top-actions">
          <button className="world-forge__save" type="button" onClick={() => onNavigate("create")}>
            <CloudUpload aria-hidden="true" size={16} strokeWidth={1.7} />
            Save
          </button>
          <button className="world-forge__run" type="button" onClick={openRunFlow}>
            <CirclePlay aria-hidden="true" size={16} strokeWidth={1.8} />
            Run
          </button>
        </div>
      </header>

      <div className="world-forge__scene">
        <span className="world-forge__preview-tag">
          <MapPinned aria-hidden="true" size={13} strokeWidth={1.8} />
          Authoring preview · not a live UE frame
        </span>

        <form
          className="world-forge__prompt"
          aria-label="Text world prompt"
          onSubmit={(event) => {
            event.preventDefault();
            generateFromPrompt();
          }}
        >
          <Sparkles aria-hidden="true" size={20} strokeWidth={1.7} />
          <input
            aria-label="World generation prompt"
            value={prompt}
            onChange={(event) => setPrompt(event.target.value)}
          />
          <button type="submit" disabled={!prompt.trim()}>
            Generate
          </button>
        </form>

        <nav className="world-forge__tool-rail" aria-label="World authoring tools">
          {TOOLS.map((tool) => {
            const Icon = tool.icon;
            const active = editor.selectedTool === tool.id;
            return (
              <button
                className={active ? "is-active" : ""}
                type="button"
                aria-label={tool.label}
                aria-pressed={active}
                title={tool.label}
                onClick={() => setEditor((current) => ({ ...current, selectedTool: tool.id }))}
                key={tool.id}
              >
                <Icon aria-hidden="true" size={22} strokeWidth={1.55} />
              </button>
            );
          })}
        </nav>

        <svg
          className="world-forge__routes"
          viewBox="0 0 1572 823"
          preserveAspectRatio="none"
          role="img"
          aria-label="Illustrative patrol route overlay"
        >
          <path d="M 72 364 C 180 420, 262 422, 344 470 S 510 500, 632 480 S 826 496, 984 552 S 1230 570, 1456 675" />
          <path d="M 622 480 C 735 430, 794 370, 922 355 S 1134 334, 1304 380" />
          {[72, 344, 632, 984, 1230, 1456].map((x, index) => (
            <circle
              cx={x}
              cy={[364, 470, 480, 552, 570, 675][index]}
              r="6"
              key={`${x}-${index}`}
            />
          ))}
        </svg>

        <span className="world-forge__spawn world-forge__spawn--west">
          <span><Bot aria-hidden="true" size={19} /></span>
          <small>Spawn</small>
        </span>
        <span className="world-forge__spawn world-forge__spawn--east">
          <span><Bot aria-hidden="true" size={19} /></span>
          <small>Spawn</small>
        </span>

        <span className="world-forge__cursor world-forge__cursor--alex">
          <strong>Alex</strong>
          <MousePointer2 aria-hidden="true" size={28} fill="currentColor" />
        </span>
        <span className="world-forge__cursor world-forge__cursor--priya">
          <strong>Priya</strong>
          <MousePointer2 aria-hidden="true" size={28} fill="currentColor" />
        </span>
        <span className="world-forge__cursor world-forge__cursor--jordan">
          <strong>Jordan</strong>
          <MousePointer2 aria-hidden="true" size={28} fill="currentColor" />
        </span>

        {inspectorOpen ? (
          <aside className="world-forge__inspector" aria-label="Road segment inspector">
            <header>
              <strong>Road Segment</strong>
              <button type="button" aria-label="Close road inspector" onClick={() => setInspectorOpen(false)}>
                <X aria-hidden="true" size={17} strokeWidth={1.7} />
              </button>
            </header>
            <label>
              <span>Name</span>
              <input
                value={editor.roadInspector.name}
                onChange={(event) => setEditor((current) => updateWorldForgeRoadInspector(current, {
                  name: event.target.value,
                }))}
              />
            </label>
            <label>
              <span>Width</span>
              <select
                value={editor.roadInspector.widthMeters}
                onChange={(event) => setEditor((current) => updateWorldForgeRoadInspector(current, {
                  widthMeters: Number(event.target.value),
                }))}
              >
                {[8, 10, 12, 14].map((width) => <option value={width} key={width}>{width.toFixed(1)} m</option>)}
              </select>
            </label>
            <label>
              <span>Material</span>
              <select
                value={editor.roadInspector.material}
                onChange={(event) => setEditor((current) => updateWorldForgeRoadInspector(current, {
                  material: event.target.value as "Asphalt" | "Concrete",
                }))}
              >
                <option value="Asphalt">Asphalt</option>
                <option value="Concrete">Concrete</option>
              </select>
            </label>
            <div className="world-forge__toggle-row">
              <span>Navigation</span>
              <button
                className={editor.roadInspector.navigationEnabled ? "is-on" : ""}
                type="button"
                role="switch"
                aria-checked={editor.roadInspector.navigationEnabled}
                onClick={() => setEditor((current) => updateWorldForgeRoadInspector(current, {
                  navigationEnabled: !current.roadInspector.navigationEnabled,
                }))}
              >
                <span aria-hidden="true" />
              </button>
            </div>
          </aside>
        ) : (
          <button className="world-forge__open-inspector" type="button" onClick={() => setInspectorOpen(true)}>
            <Layers3 aria-hidden="true" size={17} />
            Inspector
          </button>
        )}

        <div className="world-forge__asset-tray" aria-label="World asset library">
          {WORLD_FORGE_ASSETS.map((asset) => {
            const selected = editor.selectedAsset === asset.id;
            return (
              <button
                className={selected ? "is-selected" : ""}
                type="button"
                aria-pressed={selected}
                onClick={() => {
                  setEditor((current) => selectWorldForgeAsset(current, asset.id));
                  if (asset.id === "road") setInspectorOpen(true);
                }}
                key={asset.id}
              >
                <img src={asset.thumbnail} alt="" />
                <span>{asset.label}</span>
              </button>
            );
          })}
        </div>

        {!messageDismissed && (notice || warningText) ? (
          <div className="world-forge__warning" role="status">
            {notice ? <Save aria-hidden="true" size={18} /> : <AlertTriangle aria-hidden="true" size={18} />}
            <span>{notice ?? warningText}</span>
            <button
              type="button"
              aria-label="Dismiss message"
              onClick={() => {
                setNotice(null);
                setMessageDismissed(true);
              }}
            >
              <X aria-hidden="true" size={17} />
            </button>
          </div>
        ) : null}

        <button className="world-forge__refresh" type="button" onClick={onRefresh} title={command.stage.label}>
          <RefreshCw aria-hidden="true" size={15} />
          <span>{serviceReady ? "Service ready" : command.stage.label}</span>
        </button>
      </div>
    </section>
  );
}

function worldWarning({
  healthLoading,
  healthError,
  catalogLoading,
  catalogError,
  capabilities,
  capabilitiesError,
}: Pick<
  WorldCommandProps,
  | "healthLoading"
  | "healthError"
  | "catalogLoading"
  | "catalogError"
  | "capabilities"
  | "capabilitiesError"
>): string {
  if (healthLoading || catalogLoading) return "Checking SimStudio truth · authoring preview remains local";
  if (healthError) return "SimStudio API unavailable · Run opens session setup only";
  if (catalogError) return "Catalog unavailable · existing preview remains read-only";
  if (capabilitiesError || !capabilities) return "Tooling capability not reported · UE live frame not connected";
  return "UE live frame not connected · Cook status not reported";
}
