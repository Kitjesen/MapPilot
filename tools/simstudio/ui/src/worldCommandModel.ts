import type { HealthStatus } from "./api.ts";

export type WorldCommandViewId = "world" | "models" | "create" | "runs" | "evidence";

export const SIMSTUDIO_DEFAULT_VIEW = "runs" satisfies WorldCommandViewId;

export interface WorldCommandNavigationItem {
  id: WorldCommandViewId;
  label: string;
  detail: string;
}

export type WorldStageStatus =
  | "loading"
  | "offline"
  | "service-error"
  | "runtime-missing"
  | "authoring-preview";

export interface WorldCommandStage {
  status: WorldStageStatus;
  label: string;
  canShowLiveViewport: boolean;
}

export interface WorldCommandState {
  navigation: readonly WorldCommandNavigationItem[];
  stage: WorldCommandStage;
}

export interface WorldCommandStateInput {
  health: HealthStatus | null;
  healthLoading?: boolean;
  healthError?: string | null;
  verifiedVisualFrame?: boolean;
}

export const WORLD_FORGE_REGIONS = [
  "topbar",
  "prompt",
  "toolRail",
  "worldCanvas",
  "routeOverlay",
  "collaborationOverlay",
  "inspector",
  "assetTray",
  "warningToast",
] as const;

export type WorldForgeAssetId =
  | "terrain"
  | "road"
  | "warehouse"
  | "fence"
  | "pedestrian"
  | "thunderv4"
  | "sensor"
  | "mission";

export interface WorldForgeAsset {
  id: WorldForgeAssetId;
  label: string;
  thumbnail: string;
}

export const WORLD_FORGE_ASSETS: readonly WorldForgeAsset[] = [
  { id: "terrain", label: "Terrain", thumbnail: "/assets/world-forge/terrain.png" },
  { id: "road", label: "Road", thumbnail: "/assets/world-forge/road.png" },
  { id: "warehouse", label: "Warehouse", thumbnail: "/assets/world-forge/warehouse.png" },
  { id: "fence", label: "Fence", thumbnail: "/assets/world-forge/fence.png" },
  { id: "pedestrian", label: "Pedestrian", thumbnail: "/assets/world-forge/pedestrian.png" },
  { id: "thunderv4", label: "ThunderV4", thumbnail: "/assets/world-forge/thunderv4.png" },
  { id: "sensor", label: "Sensor", thumbnail: "/assets/world-forge/sensor.png" },
  { id: "mission", label: "Mission", thumbnail: "/assets/world-forge/mission.png" },
];

export type WorldForgeToolId = "select" | "terrain" | "objects" | "agents" | "layers";

export interface WorldForgeRoadInspector {
  name: string;
  widthMeters: number;
  material: "Asphalt" | "Concrete";
  navigationEnabled: boolean;
}

export interface WorldForgeEditorState {
  selectedTool: WorldForgeToolId;
  selectedAsset: WorldForgeAssetId;
  roadInspector: WorldForgeRoadInspector;
  runtimeTruth: {
    source: "authoring-preview";
    liveUeFrame: false;
  };
}

export function createWorldForgeEditorState(): WorldForgeEditorState {
  return {
    selectedTool: "select",
    selectedAsset: "road",
    roadInspector: {
      name: "Road Segment 12",
      widthMeters: 10,
      material: "Asphalt",
      navigationEnabled: true,
    },
    runtimeTruth: {
      source: "authoring-preview",
      liveUeFrame: false,
    },
  };
}

export function selectWorldForgeAsset(
  state: WorldForgeEditorState,
  selectedAsset: WorldForgeAssetId,
): WorldForgeEditorState {
  return { ...state, selectedAsset };
}

export function updateWorldForgeRoadInspector(
  state: WorldForgeEditorState,
  patch: Partial<WorldForgeRoadInspector>,
): WorldForgeEditorState {
  return {
    ...state,
    roadInspector: { ...state.roadInspector, ...patch },
  };
}

export const WORLD_COMMAND_NAVIGATION: readonly WorldCommandNavigationItem[] = [
  { id: "world", label: "World Preview", detail: "Web authoring preview · not UE5 runtime" },
  { id: "models", label: "Models", detail: "Packages, robots, worlds" },
  { id: "create", label: "Create", detail: "Factory terrain and elements" },
  { id: "runs", label: "Runs", detail: "Session, runtime, recording" },
  { id: "evidence", label: "Evidence", detail: "Replay and artifacts" },
];

export function buildWorldCommandState(
  input: WorldCommandStateInput,
): WorldCommandState {
  return {
    navigation: WORLD_COMMAND_NAVIGATION,
    stage: worldStageFromHealth(input),
  };
}

function worldStageFromHealth(input: WorldCommandStateInput): WorldCommandStage {
  const { health } = input;

  if (health === null && input.healthLoading === true) {
    return {
      status: "loading",
      label: "Checking SimStudio service",
      canShowLiveViewport: false,
    };
  }

  if (health === null && input.healthError) {
    return {
      status: "service-error",
      label: "SimStudio service unavailable",
      canShowLiveViewport: false,
    };
  }

  if (health === null) {
    return {
      status: "offline",
      label: "SimStudio service offline",
      canShowLiveViewport: false,
    };
  }

  if (health.status !== "ok") {
    return {
      status: "service-error",
      label: "SimStudio service is not healthy",
      canShowLiveViewport: false,
    };
  }

  if (!health.runtime_bound) {
    return {
      status: "runtime-missing",
      label: "Run service not bound",
      canShowLiveViewport: false,
    };
  }

  return {
    status: "authoring-preview",
    label: "Web authoring preview · UE5 runtime is separate",
    canShowLiveViewport: false,
  };
}
