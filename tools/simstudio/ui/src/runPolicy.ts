import type { RunOperation } from "./api.ts";

const OPERATIONS_BY_STATUS: Readonly<Record<string, readonly RunOperation[]>> = {
  CREATED: ["prepare", "stop"],
  READY: ["start", "reset", "stop"],
  RUNNING: ["pause", "reset", "stop"],
  PAUSED: ["start", "reset", "stop"],
  FAILED: ["stop"],
  STOPPED: [],
};

export function availableRunOperations(status: string): RunOperation[] {
  return [...(OPERATIONS_BY_STATUS[status] ?? [])];
}
