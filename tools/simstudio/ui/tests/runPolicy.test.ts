import assert from "node:assert/strict";
import test from "node:test";

import { availableRunOperations } from "../src/runPolicy.ts";

test("run controls expose only lifecycle operations accepted by the current state", () => {
  assert.deepEqual(availableRunOperations("CREATED"), ["prepare", "stop"]);
  assert.deepEqual(availableRunOperations("READY"), ["start", "reset", "stop"]);
  assert.deepEqual(availableRunOperations("RUNNING"), ["pause", "reset", "stop"]);
  assert.deepEqual(availableRunOperations("PAUSED"), ["start", "reset", "stop"]);
  assert.deepEqual(availableRunOperations("FAILED"), ["stop"]);
  assert.deepEqual(availableRunOperations("STOPPED"), []);
  assert.deepEqual(availableRunOperations("UNKNOWN"), []);
});
