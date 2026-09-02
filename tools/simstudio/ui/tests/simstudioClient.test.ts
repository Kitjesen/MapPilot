import assert from "node:assert/strict";
import test from "node:test";

import { API_PREFIX, SimStudioApiError, SimStudioClient, simApiPath } from "../src/api.ts";

test("the client keeps every request inside the SimStudio v1 interface", async () => {
  const requested: string[] = [];
  const client = new SimStudioClient(async (input) => {
    requested.push(String(input));
    return new Response(
      JSON.stringify({
        ok: true,
        result: { schema: "lingtu.sim.catalog-list.v1", packages: [] },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  const catalog = await client.listPackages();

  assert.deepEqual(catalog.packages, []);
  assert.deepEqual(requested, [`${API_PREFIX}/packages`]);
  assert.equal(simApiPath("/runs/demo/readiness"), `${API_PREFIX}/runs/demo/readiness`);
  assert.throws(() => simApiPath("/api/v1/health"), SimStudioApiError);
  assert.throws(() => simApiPath("https://example.com/api/sim/v1/health"), SimStudioApiError);
});

test("a composed session can become a CAS-controlled run with readiness and artifacts", async () => {
  const calls: Array<{ url: string; method: string; body: unknown; key: string | null }> = [];
  const replies: unknown[] = [
    {
      schema: "lingtu.sim.studio.session-draft.v1",
      kind: "session_draft",
      id: "draft01",
      revision: 1,
      created_at: "2026-08-09T00:00:00Z",
      updated_at: "2026-08-09T00:00:00Z",
      status: "DRAFT",
      payload: { intent: {} },
    },
    {
      schema: "lingtu.sim.studio.bundle.v1",
      kind: "bundle",
      id: "bundle01",
      revision: 1,
      created_at: "2026-08-09T00:00:01Z",
      updated_at: "2026-08-09T00:00:01Z",
      status: "COMPOSED",
      payload: { session_id: "yard_run" },
    },
    {
      schema: "lingtu.sim.studio.run.v1",
      kind: "run",
      id: "run01",
      revision: 1,
      created_at: "2026-08-09T00:00:02Z",
      updated_at: "2026-08-09T00:00:02Z",
      status: "CREATED",
      payload: { bundle_id: "bundle01", launch_profile: "visual" },
    },
    {
      schema: "lingtu.sim.studio.run.v1",
      kind: "run",
      id: "run01",
      revision: 2,
      created_at: "2026-08-09T00:00:02Z",
      updated_at: "2026-08-09T00:00:03Z",
      status: "READY",
      payload: { bundle_id: "bundle01", launch_profile: "visual" },
    },
    {
      run_id: "run01",
      status: "READY",
      revision: 2,
      ready: true,
      readiness: { physics: "ready", visual: "ready", sensors: "ready" },
      sensors: { imu: "ready", mid360: "ready" },
    },
    [
      {
        artifact_id: "artifact01",
        run_id: "run01",
        path: "logs/visual-readiness.json",
        kind: "file",
        size: 240,
        sha256: "c".repeat(64),
      },
    ],
  ];
  const client = new SimStudioClient(async (input, init) => {
    calls.push({
      url: String(input),
      method: init?.method ?? "GET",
      body: init?.body === undefined ? undefined : JSON.parse(String(init.body)),
      key: new Headers(init?.headers).get("Idempotency-Key"),
    });
    const result = replies.shift();
    return new Response(JSON.stringify({ ok: true, result }), {
      status: 200,
      headers: { "content-type": "application/json" },
    });
  });
  const intent = {
    schema: "lingtu.sim.session-intent.v1",
    session: {
      session_id: "yard_run",
      mujoco_version: "3.10.0",
      seed: 17,
      world: "factory_park_hf@1.0.0",
      robots: [],
      runtime: { backend: "mujoco", mode: "unreal", required_bindings: ["physics", "visual"] },
    },
  };

  const draft = await client.createDraft(intent, "draft-key");
  const bundle = await client.composeDraft(draft.id, draft.revision, "compose-key");
  const run = await client.createRun(bundle.id, "visual", "run-key");
  const prepared = await client.operateRun(run.id, "prepare", run.revision, "prepare-key");
  const readiness = await client.getRunReadiness(run.id);
  const artifacts = await client.listArtifacts(run.id);

  assert.equal(prepared.status, "READY");
  assert.equal(readiness.readiness.visual, "ready");
  assert.equal(artifacts[0]?.path, "logs/visual-readiness.json");
  assert.deepEqual(calls, [
    {
      url: `${API_PREFIX}/session-drafts`,
      method: "POST",
      body: { intent },
      key: "draft-key",
    },
    {
      url: `${API_PREFIX}/session-drafts/draft01/compose`,
      method: "POST",
      body: { revision: 1 },
      key: "compose-key",
    },
    {
      url: `${API_PREFIX}/runs`,
      method: "POST",
      body: { bundle_id: "bundle01", launch_profile: "visual" },
      key: "run-key",
    },
    {
      url: `${API_PREFIX}/runs/run01/prepare`,
      method: "POST",
      body: { revision: 1 },
      key: "prepare-key",
    },
    {
      url: `${API_PREFIX}/runs/run01/readiness`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/runs/run01/artifacts`,
      method: "GET",
      body: undefined,
      key: null,
    },
  ]);
});

test("the client exposes imports, read models, capabilities, and safe artifact preview", async () => {
  const calls: Array<{ url: string; method: string; body: unknown; key: string | null }> = [];
  const replies: unknown[] = [
    {
      schema: "lingtu.sim.studio.capabilities.v1",
      api_version: "v1",
      api_prefix: API_PREFIX,
      field_isolated: true,
      read_models: {
        imports: { list: `${API_PREFIX}/imports` },
        session_drafts: { list: `${API_PREFIX}/session-drafts` },
        bundles: { list: `${API_PREFIX}/bundles` },
        run_artifacts: {
          list: `${API_PREFIX}/runs/{run_id}/artifacts`,
          preview: `${API_PREFIX}/runs/{run_id}/artifacts/{artifact_id}/preview`,
        },
      },
      artifact_preview: {
        addressing: "opaque_artifact_id",
        raw_path_input: false,
        max_bytes: 1024,
      },
      schema_document: {
        href: `${API_PREFIX}/schema`,
        format: "openapi-3.1",
      },
    },
    [],
    {
      schema: "lingtu.sim.studio.import-job.v1",
      kind: "import_job",
      id: "import01",
      revision: 1,
      created_at: "2026-08-09T00:00:00Z",
      updated_at: "2026-08-09T00:00:00Z",
      status: "QUEUED",
      payload: { kind: "world" },
    },
    {
      schema: "lingtu.sim.studio.import-job.v1",
      kind: "import_job",
      id: "import01",
      revision: 2,
      created_at: "2026-08-09T00:00:00Z",
      updated_at: "2026-08-09T00:00:01Z",
      status: "PROMOTED",
      payload: { kind: "world" },
    },
    [],
    [],
    {
      path: "logs/episode_result.json",
      size: 18,
      sha256: "d".repeat(64),
      mime_type: "application/json",
      previewable: true,
      encoding: "utf-8",
      content: "{\"ok\":true}",
      truncated: false,
      format: "json",
      artifact_id: "a".repeat(32),
      run_id: "run01",
    },
  ];
  const client = new SimStudioClient(async (input, init) => {
    calls.push({
      url: String(input),
      method: init?.method ?? "GET",
      body: init?.body === undefined ? undefined : JSON.parse(String(init.body)),
      key: new Headers(init?.headers).get("Idempotency-Key"),
    });
    return new Response(JSON.stringify({ ok: true, result: replies.shift() }), {
      status: 200,
      headers: { "content-type": "application/json" },
    });
  });

  const capabilities = await client.capabilities();
  const imports = await client.listImports();
  const created = await client.createImport(
    "world",
    "incoming/factory.zip",
    { package_ref: "factory_park_hf@1.0.0" },
    "import-key",
  );
  const promoted = await client.promoteImport(created.id, "promote-key");
  const drafts = await client.listDrafts();
  const bundles = await client.listBundles();
  const preview = await client.previewArtifact("run01", "a".repeat(32), 512);

  assert.equal(capabilities.artifact_preview.raw_path_input, false);
  assert.deepEqual(imports, []);
  assert.equal(promoted.status, "PROMOTED");
  assert.deepEqual(drafts, []);
  assert.deepEqual(bundles, []);
  assert.equal(preview.previewable, true);
  assert.deepEqual(calls, [
    {
      url: `${API_PREFIX}/capabilities`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/imports`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/imports`,
      method: "POST",
      body: {
        kind: "world",
        source: { entry: "incoming/factory.zip" },
        request: { package_ref: "factory_park_hf@1.0.0" },
      },
      key: "import-key",
    },
    {
      url: `${API_PREFIX}/imports/import01/promote`,
      method: "POST",
      body: undefined,
      key: "promote-key",
    },
    {
      url: `${API_PREFIX}/session-drafts`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/bundles`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/runs/run01/artifacts/${"a".repeat(32)}/preview?max_bytes=512`,
      method: "GET",
      body: undefined,
      key: null,
    },
  ]);
});

test("the client uploads a local archive as binary into the managed source inbox", async () => {
  const calls: Array<{ url: string; method: string; headers: Headers; body: BodyInit | null | undefined }> = [];
  const sourceId = "source01";
  const client = new SimStudioClient(async (input, init) => {
    calls.push({
      url: String(input),
      method: init?.method ?? "GET",
      headers: new Headers(init?.headers),
      body: init?.body,
    });
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.inbox-source.v1",
          source_id: sourceId,
          entry: `objects/${sourceId}.zip`,
          original_name: "robot.zip",
          bytes: 4,
          archive_format: "zip",
        },
      }),
      { status: 201, headers: { "content-type": "application/json" } },
    );
  });
  const file = new File([new Uint8Array([1, 2, 3, 4])], "robot.zip", {
    type: "application/zip",
  });

  const source = await client.uploadSource(file);

  assert.equal(source.source_id, sourceId);
  assert.equal(calls.length, 1);
  assert.equal(calls[0]?.url, `${API_PREFIX}/inbox/uploads`);
  assert.equal(calls[0]?.method, "POST");
  assert.equal(calls[0]?.headers.get("X-SimStudio-Filename"), "robot.zip");
  assert.equal(calls[0]?.headers.get("content-type"), "application/octet-stream");
  assert.deepEqual(
    [...new Uint8Array(await (calls[0]?.body as Blob).arrayBuffer())],
    [1, 2, 3, 4],
  );
});

test("the client obtains importer-owned request templates", async () => {
  const requested: string[] = [];
  const client = new SimStudioClient(async (input) => {
    requested.push(String(input));
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.import-contract.v1",
          kind: "robot",
          source_entry_owned_by: "simstudio",
          request_schema: {
            id: "lingtu.sim.robot-import-request.v1",
            path: "sim/contracts/schemas/robot-import.v1.json",
            sha256: "f".repeat(64),
          },
          request_template: {
            schema: "lingtu.sim.robot-import-request.v1",
            id: "new_robot",
          },
        },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  const contract = await client.getImportContract("robot");

  assert.equal(contract.request_template.id, "new_robot");
  assert.deepEqual(requested, [`${API_PREFIX}/import-contracts/robot`]);
});

test("the client requests source inspection by opaque source ID", async () => {
  const requested: string[] = [];
  const sourceId = "source01";
  const client = new SimStudioClient(async (input) => {
    requested.push(String(input));
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.source-inspection.v1",
          source: {
            source_id: sourceId,
            entry: `objects/${sourceId}.zip`,
            bytes: 64,
            archive_format: "zip",
          },
          summary: { files: 2, total_bytes: 32 },
          candidates: {
            robot_models: [{ path: "robot.xml", size: 20, sha256: "b".repeat(64), format: "mjcf" }],
            licenses: [{ path: "LICENSE.txt", size: 12, sha256: "c".repeat(64) }],
            heightmaps: [],
            meshes: [],
            textures: [],
          },
          recommendations: {
            robot: { source_format: "mjcf", source_model: "robot.xml", license_file: "LICENSE.txt" },
            world: null,
          },
          files: [],
        },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  const inspection = await client.inspectSource(sourceId);

  assert.equal(inspection.recommendations.robot?.source_model, "robot.xml");
  assert.deepEqual(requested, [`${API_PREFIX}/inbox/sources/${sourceId}/inspection`]);
});

test("the client reads validated recording status without accepting a path", async () => {
  const requested: string[] = [];
  const client = new SimStudioClient(async (input) => {
    requested.push(String(input));
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.recording.v1",
          recording_id: "a".repeat(32),
          run_id: "run01",
          state: "VALID",
          source_run_id: "run01",
          session_id: "studio-session",
          model_generation: 2,
          reset_generation: { start: 3, end: 3 },
          frame_count: 120,
          duration_ns: 2_000_000_000,
          terminal_state: "STOPPED",
          event_order: ["truth_snapshot"],
          replay: {
            deterministic: true,
            visual: true,
            clock_authority: "recorded_mujoco",
          },
          artifacts: {
            manifest: "simulation-recording.json",
            timeline: "simulation-timeline.jsonl",
          },
          diagnostics: [],
        },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  const recording = await client.inspectRecording("run01");

  assert.equal(recording.frame_count, 120);
  assert.equal(recording.replay.clock_authority, "recorded_mujoco");
  assert.deepEqual(requested, [`${API_PREFIX}/runs/run01/recording`]);
});

test("the client pages a validated recording timeline through the exact run-owned URL", async () => {
  const requested: string[] = [];
  const client = new SimStudioClient(async (input) => {
    requested.push(String(input));
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.recording-timeline.v1",
          recording_id: "a".repeat(32),
          run_id: "run 01",
          session_id: "studio-session",
          frame_count: 240,
          duration_ns: 4_780_000_000,
          page: {
            offset: 100,
            limit: 100,
            returned: 1,
            next_offset: 101,
          },
          frames: [
            {
              frame_index: 100,
              relative_time_ns: 2_000_000_000,
              sim_time_ns: 3_000_000_000,
              sequence: 110,
              physics_step: 880,
              model_generation: 2,
              reset_generation: 3,
              body_count: 12,
              joint_count: 18,
              actuator_count: 12,
              sensor_count: 3,
              has_command: true,
              scenario_event_count: 1,
              sensor_metadata_count: 3,
              sensor_payload_count: 2,
              sensor_payload_bytes: 6144,
              lifecycle_evidence_count: 0,
            },
          ],
        },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  const timeline = await client.getRecordingTimeline("run 01", 100, 100);

  assert.equal(timeline.frames[0]?.frame_index, 100);
  assert.equal(timeline.frames[0]?.sensor_payload_count, 2);
  assert.equal(timeline.frames[0]?.sensor_payload_bytes, 6144);
  assert.deepEqual(requested, [
    `${API_PREFIX}/runs/run%2001/recording/timeline?offset=100&limit=100`,
  ]);
});

test("the client reads one recording frame by encoded run identity instead of a raw path", async () => {
  const requested: string[] = [];
  const client = new SimStudioClient(async (input) => {
    requested.push(String(input));
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.recording-frame.v1",
          recording_id: "a".repeat(32),
          run_id: "run/01",
          session_id: "studio-session",
          frame_index: 17,
          relative_time_ns: 340_000_000,
          snapshot: {
            event: "snapshot",
            sequence: 27,
            physics_step: 216,
            sim_time_ns: 1_340_000_000,
            model_generation: 2,
            reset_generation: 3,
            bodies: [
              {
                stable_id: "robot_01/base_link",
                instance_id: "robot_01",
                frame_id: "base_link",
                position_m: [1.25, -0.5, 0.62],
                quaternion_wxyz: [1, 0, 0, 0],
              },
            ],
            joints: [],
            actuators: [],
            sensors: [],
          },
          command: null,
          metadata: {},
          scenario_events: [],
          sensor_metadata: [],
          sensor_payloads: [
            {
              schema: "lingtu.sim.sensor-payload-ref.v1",
              payload_index: 0,
              session_id: "studio-session",
              model_generation: 2,
              reset_generation: 3,
              sensor_id: "robot_01.front_depth",
              stream_kind: "depth",
              encoding: "32FC1",
              media_type: "application/vnd.lingtu.depth-f32",
              sample_sequence: 9,
              sample_time_ns: 1_340_000_000,
              sha256: "c".repeat(64),
              bytes: 16,
              metadata: { width: 2, height: 2 },
            },
          ],
          lifecycle_evidence: [],
        },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  const frame = await client.getRecordingFrame("run/01", 17);

  assert.deepEqual(frame.snapshot.bodies[0]?.position_m, [1.25, -0.5, 0.62]);
  assert.equal(frame.sensor_payloads[0]?.sensor_id, "robot_01.front_depth");
  assert.deepEqual(requested, [
    `${API_PREFIX}/runs/run%2F01/recording/frames/17`,
  ]);
});

test("the client rejects unbounded recording page and frame coordinates before fetching", () => {
  let fetchCount = 0;
  const client = new SimStudioClient(async () => {
    fetchCount += 1;
    return new Response(JSON.stringify({ ok: true, result: {} }));
  });

  assert.throws(
    () => client.getRecordingTimeline("run01", -1, 100),
    SimStudioApiError,
  );
  assert.throws(
    () => client.getRecordingTimeline("run01", 0, 201),
    SimStudioApiError,
  );
  assert.throws(
    () => client.getRecordingFrame("run01", 1.5),
    SimStudioApiError,
  );
  assert.equal(fetchCount, 0);
});

test("the client controls a recording window with run revision guards", async () => {
  const calls: Array<{ url: string; body: unknown; key: string | null }> = [];
  const client = new SimStudioClient(async (input, init) => {
    calls.push({
      url: String(input),
      body: JSON.parse(String(init?.body)),
      key: new Headers(init?.headers).get("Idempotency-Key"),
    });
    return new Response(
      JSON.stringify({
        ok: true,
        result: {
          schema: "lingtu.sim.studio.run.v1",
          kind: "run",
          id: "run01",
          revision: 5,
          created_at: "2026-08-10T00:00:00Z",
          updated_at: "2026-08-10T00:00:01Z",
          status: "RUNNING",
          payload: { recording: { state: "CAPTURING" } },
        },
      }),
      { status: 200, headers: { "content-type": "application/json" } },
    );
  });

  await client.operateRecording("run01", "start", 4, "recording-key");

  assert.deepEqual(calls, [
    {
      url: `${API_PREFIX}/runs/run01/recording/start`,
      body: { revision: 4 },
      key: "recording-key",
    },
  ]);
});

test("the client validates declarative scene elements without process controls", async () => {
  const calls: Array<{ url: string; method: string; body: unknown }> = [];
  const replies = [
    {
      schema: "lingtu.sim.studio.scene-tool-catalog.v1",
      scene_tool: "factory-park-hf",
      world_package: "factory_park_hf@1.0.0",
      element_batch_schema: "lingtu.sim.factory-park-element-batch.v1",
      read_only: true,
      executes_runtime: false,
      layout_digest: "a".repeat(64),
      element_types: {},
      surfaces: [],
      spawn: { position_xy_m: [0, -76], clearance_radius_m: 4 },
    },
    {
      schema: "lingtu.sim.studio.scene-tool-validation.v1",
      scene_tool: "factory-park-hf",
      world_package: "factory_park_hf@1.0.0",
      valid: true,
      batch_id: "studio_layout",
      digest: "b".repeat(64),
      layout_digest: "a".repeat(64),
      stable_ids: ["element__studio_layout__cone_01"],
      elements: [
        {
          stable_id: "element__studio_layout__cone_01",
          element_type: "traffic_cone",
          authority: "PhysicsShared",
        },
      ],
      diagnostics: [],
    },
  ];
  const client = new SimStudioClient(async (input, init) => {
    calls.push({
      url: String(input),
      method: init?.method ?? "GET",
      body: init?.body === undefined ? undefined : JSON.parse(String(init.body)),
    });
    return new Response(JSON.stringify({ ok: true, result: replies.shift() }), {
      status: 200,
      headers: { "content-type": "application/json" },
    });
  });
  const batch = {
    schema: "lingtu.sim.factory-park-element-batch.v1" as const,
    batch_id: "studio_layout",
    description: "",
    elements: [
      {
        instance_key: "cone_01",
        element_type: "traffic_cone",
        surface_id: "parking_apron",
        position_xy_m: [24, -44] as [number, number],
        yaw_deg: 0,
      },
    ],
  };

  await client.getFactoryParkSceneCatalog();
  const validation = await client.validateFactoryParkElementBatch(batch);

  assert.equal(validation.valid, true);
  assert.deepEqual(calls, [
    {
      url: `${API_PREFIX}/scene-tools/factory-park-hf/catalog`,
      method: "GET",
      body: undefined,
    },
    {
      url: `${API_PREFIX}/scene-tools/factory-park-hf/element-batches/validate`,
      method: "POST",
      body: batch,
    },
  ]);
});

test("the client persists scene drafts through opaque ids and revision guards", async () => {
  const calls: Array<{
    url: string;
    method: string;
    body: unknown;
    key: string | null;
  }> = [];
  const batch = {
    schema: "lingtu.sim.factory-park-element-batch.v1" as const,
    batch_id: "studio_layout",
    description: "persisted",
    elements: [
      {
        instance_key: "cone_01",
        element_type: "traffic_cone",
        surface_id: "parking_apron",
        position_xy_m: [24, -44] as [number, number],
        yaw_deg: 0,
      },
    ],
  };
  const record = {
    schema: "lingtu.sim.studio.scene-draft.v1",
    kind: "scene_draft",
    id: "a".repeat(32),
    revision: 1,
    created_at: "2026-08-10T00:00:00Z",
    updated_at: "2026-08-10T00:00:00Z",
    status: "draft",
    payload: {
      schema: "lingtu.sim.studio.scene-draft-payload.v1",
      scene_tool: "factory-park-hf",
      world_package: "factory_park_hf@1.0.0",
      layout_digest: "b".repeat(64),
      batch_digest: "c".repeat(64),
      batch,
      validation: {},
    },
  };
  const published = {
    schema: "lingtu.sim.studio.scene-publication.v1",
    scene_draft: { ...record, revision: 2, status: "published" },
    publication: {
      schema: "lingtu.sim.studio.world-publication.v1",
      package: { kind: "world", id: "safety_layout", version: "1.0.0", ref: "safety_layout@1.0.0" },
      package_root: "sim/packages/worlds/safety_layout/1.0.0",
      qualification_path: "sim/evaluation/package_qualifications/world/safety_layout/1.0.0.qualification.json",
      content_digest: "d".repeat(64),
    },
    source: {
      scene_draft_id: record.id,
      scene_draft_revision: 1,
      batch_digest: "c".repeat(64),
      base_layout_digest: "b".repeat(64),
    },
  };
  const replies = [record, [record], record, { ...record, revision: 2 }, published];
  const client = new SimStudioClient(async (input, init) => {
    const headers = new Headers(init?.headers);
    calls.push({
      url: String(input),
      method: init?.method ?? "GET",
      body: init?.body === undefined ? undefined : JSON.parse(String(init.body)),
      key: headers.get("Idempotency-Key"),
    });
    return new Response(JSON.stringify({ ok: true, result: replies.shift() }), {
      status: 200,
      headers: { "content-type": "application/json" },
    });
  });

  const created = await client.createSceneDraft(batch, "scene-create");
  await client.listSceneDrafts();
  await client.getSceneDraft(created.id);
  await client.updateSceneDraft(created.id, created.revision, batch, "scene-update");
  await client.publishSceneDraft(
    created.id,
    created.revision,
    { id: "safety_layout", version: "1.0.0", description: "Safety layout" },
    "scene-publish",
  );

  assert.deepEqual(calls, [
    {
      url: `${API_PREFIX}/scene-drafts`,
      method: "POST",
      body: { scene_tool: "factory-park-hf", batch },
      key: "scene-create",
    },
    {
      url: `${API_PREFIX}/scene-drafts`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/scene-drafts/${"a".repeat(32)}`,
      method: "GET",
      body: undefined,
      key: null,
    },
    {
      url: `${API_PREFIX}/scene-drafts/${"a".repeat(32)}`,
      method: "PUT",
      body: { revision: 1, batch },
      key: "scene-update",
    },
    {
      url: `${API_PREFIX}/scene-drafts/${"a".repeat(32)}/publish`,
      method: "POST",
      body: {
        revision: 1,
        package: { id: "safety_layout", version: "1.0.0", description: "Safety layout" },
      },
      key: "scene-publish",
    },
  ]);
});
