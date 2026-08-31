#!/usr/bin/env node

const host = process.env.ROBOT_HOST || '127.0.0.1:5050'
const base = (process.env.GATEWAY_URL || `http://${host}`).replace(/\/+$/, '')
const allowDegradedReady = process.env.ALLOW_DEGRADED_READY === '1'

const requiredBootstrapKeys = [
  'capabilities',
  'control',
  'links',
  'localization',
  'map',
  'media',
  'mission',
  'navigation',
  'robot',
  'safety',
  'scene',
  'session',
]

const requiredLinks = [
  'bootstrap',
  'events',
  'health',
  'session',
  'navigation_status',
  'navigation_plan',
  'navigation_cancel',
  'goal',
  'stop',
  'maps',
  'map_activate',
  'map_save',
  'map_points',
  'slam_status',
  'slam_relocalize',
  'routecheck_latest',
  'real_runtime_evidence_latest',
  'runtime_dataflow',
  'runtime_dataflow_topic',
  'runtime_dataflow_subscribe',
  'field_check',
  'inspection_acceptance',
]

async function request(path, { json = true, method = 'GET', body = undefined } = {}) {
  const url = new URL(path, `${base}/`)
  const res = await fetch(url, {
    method,
    headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  })
  const text = await res.text()
  let parsedBody = text
  if (json && text) {
    try {
      parsedBody = JSON.parse(text)
    } catch (err) {
      throw new Error(`${path} returned non-JSON: ${err.message}`)
    }
  }
  return { status: res.status, body: parsedBody }
}

function parseSseBlock(block) {
  const data = block
    .split(/\r?\n/)
    .filter(line => line.startsWith('data:'))
    .map(line => line.slice(5).trimStart())
    .join('\n')
  if (!data) return null
  try {
    return JSON.parse(data)
  } catch {
    return { raw: data }
  }
}

async function readFirstSseEvent(path, { timeoutMs = 5000 } = {}) {
  const url = new URL(path, `${base}/`)
  const controller = new AbortController()
  const timeout = setTimeout(() => controller.abort(), timeoutMs)
  let reader
  try {
    const res = await fetch(url, {
      headers: { Accept: 'text/event-stream' },
      signal: controller.signal,
    })
    if (!res.body) return { status: res.status, body: null }
    reader = res.body.getReader()
    const decoder = new TextDecoder()
    let buffer = ''
    while (true) {
      const { done, value } = await reader.read()
      if (done) break
      buffer += decoder.decode(value, { stream: true })
      const boundary = buffer.indexOf('\n\n')
      if (boundary >= 0) {
        return { status: res.status, body: parseSseBlock(buffer.slice(0, boundary)) }
      }
    }
    return { status: res.status, body: null }
  } finally {
    clearTimeout(timeout)
    if (reader) {
      try {
        await reader.cancel()
      } catch {
        // The AbortController may already have closed the stream.
      }
    }
    controller.abort()
  }
}

function missingKeys(obj, keys) {
  return keys.filter(key => obj == null || !(key in obj))
}

function ensure(condition, failures, message) {
  if (!condition) failures.push(message)
}

const failures = []
const requiredRuntimeTopics = [
  '/nav/odometry',
  '/nav/map_cloud',
  '/nav/global_path',
  '/nav/local_path',
  '/nav/cmd_vel',
]
const requiredRuntimeStages = [
  'slam_or_relayed_localization_map',
  'global_planning',
  'local_planning_and_following',
  'command_boundary',
]

try {
  const root = await request('/', { json: false })
  ensure(root.status === 200, failures, `GET / expected 200, got ${root.status}`)

  const bootstrap = await request('/api/v1/app/bootstrap')
  ensure(bootstrap.status === 200, failures, `GET /api/v1/app/bootstrap expected 200, got ${bootstrap.status}`)

  const bootstrapBody = bootstrap.body
  const missingBootstrap = missingKeys(bootstrapBody, requiredBootstrapKeys)
  ensure(missingBootstrap.length === 0, failures, `bootstrap missing keys: ${missingBootstrap.join(', ')}`)

  const links = bootstrapBody.links || {}
  const missingLinks = missingKeys(links, requiredLinks)
  ensure(missingLinks.length === 0, failures, `bootstrap.links missing keys: ${missingLinks.join(', ')}`)

  const health = await request('/api/v1/health')
  ensure(health.status === 200, failures, `GET /api/v1/health expected 200, got ${health.status}`)

  const routecheckPath = links.routecheck_latest || '/api/v1/diagnostics/routecheck/latest'
  const routecheck = await request(routecheckPath)
  ensure(routecheck.status === 200, failures, `GET ${routecheckPath} expected 200, got ${routecheck.status}`)
  ensure(typeof routecheck.body?.ok === 'boolean', failures, 'routecheck_latest missing boolean ok')
  ensure(typeof routecheck.body?.count === 'number', failures, 'routecheck_latest missing numeric count')
  ensure(typeof routecheck.body?.artifacts_root === 'string', failures, 'routecheck_latest missing artifacts_root')

  const evidencePath = links.real_runtime_evidence_latest || '/api/v1/diagnostics/real-runtime-evidence/latest'
  const realEvidence = await request(evidencePath)
  ensure(realEvidence.status === 200, failures, `GET ${evidencePath} expected 200, got ${realEvidence.status}`)
  ensure(typeof realEvidence.body?.ok === 'boolean', failures, 'real_runtime_evidence_latest missing boolean ok')

  const dataflowPath = links.runtime_dataflow || '/api/v1/runtime/dataflow'
  const dataflow = await request(dataflowPath)
  ensure(dataflow.status === 200, failures, `GET ${dataflowPath} expected 200, got ${dataflow.status}`)
  ensure(
    dataflow.body?.transport_layers?.module_port_bus?.primary === true,
    failures,
    'runtime_dataflow should use ModulePort bus as primary',
  )
  ensure(
    dataflow.body?.control_boundary?.arbitrary_publish_supported === false,
    failures,
    'runtime_dataflow should not support arbitrary publish',
  )
  const declaredRuntimeTopics = Array.isArray(dataflow.body?.topics) ? dataflow.body.topics : []
  ensure(Array.isArray(dataflow.body?.topics), failures, 'runtime_dataflow missing topics array')
  ensure(Array.isArray(dataflow.body?.stage_evidence), failures, 'runtime_dataflow missing stage_evidence array')
  const runtimeTopics = new Set(declaredRuntimeTopics.map(topic => topic?.topic))
  for (const topic of requiredRuntimeTopics) {
    ensure(runtimeTopics.has(topic), failures, `runtime_dataflow missing required topic ${topic}`)
  }
  const runtimeStages = new Set((dataflow.body?.stage_evidence || []).map(stage => stage?.name))
  for (const stage of requiredRuntimeStages) {
    ensure(runtimeStages.has(stage), failures, `runtime_dataflow missing required stage ${stage}`)
  }
  ensure(
    dataflow.body?.stage_evidence?.some(stage => stage?.name === 'global_planning'),
    failures,
    'runtime_dataflow stage_evidence missing global_planning stage',
  )
  const detailBase = links.runtime_dataflow_topic || '/api/v1/runtime/dataflow/topic'
  const runtimeDataflowSubscribePath = links.runtime_dataflow_subscribe || '/api/v1/runtime/dataflow/subscribe'
  const readableRuntimeTopics = new Set(requiredRuntimeTopics)
  const realtimeRuntimeTopics = new Set()
  for (const entry of declaredRuntimeTopics) {
    const topic = entry?.topic
    if (typeof topic !== 'string' || !topic) continue
    const inspection = entry?.inspection || {}
    const streamInterfaces = Array.isArray(inspection?.stream_interfaces) ? inspection.stream_interfaces : []
    const hasGatewaySse = streamInterfaces.some(item => item?.transport === 'gateway_sse' && typeof item?.event_type === 'string')
    if (
      inspection?.payload_available === true
      || inspection?.module_stats_available === true
      || streamInterfaces.length > 0
      || entry?.observability?.observable === true
    ) {
      readableRuntimeTopics.add(topic)
    }
    if (hasGatewaySse) {
      realtimeRuntimeTopics.add(topic)
    }
  }
  ensure(realtimeRuntimeTopics.size > 0, failures, 'runtime_dataflow did not declare any Gateway SSE streams')
  for (const topic of readableRuntimeTopics) {
    const sep = detailBase.includes('?') ? '&' : '?'
    const detailPath = `${detailBase}${sep}${new URLSearchParams({ topic })}`
    const detail = await request(detailPath)
    ensure(detail.status === 200, failures, `GET ${detailPath} expected 200, got ${detail.status}`)
    ensure(detail.body?.inspection?.arbitrary_publish_supported === false, failures, 'runtime_dataflow_topic should not support arbitrary publish')
  }
  for (const topic of realtimeRuntimeTopics) {
    const subscription = await request(runtimeDataflowSubscribePath, {
      method: 'POST',
      body: { selector: topic, transport: 'gateway_sse' },
    })
    ensure(
      subscription.status === 200,
      failures,
      `POST ${runtimeDataflowSubscribePath} expected 200 for ${topic}, got ${subscription.status}`,
    )
    ensure(
      subscription.body?.schema_version === 'lingtu.runtime_dataflow_subscription.v1',
      failures,
      `runtime_dataflow_subscribe missing canonical schema version for ${topic}`,
    )
    ensure(subscription.body?.ok === true, failures, `runtime_dataflow_subscribe should be ok for ${topic}`)
    ensure(subscription.body?.read_only === true, failures, `runtime_dataflow_subscribe should be read-only for ${topic}`)
    ensure(
      subscription.body?.arbitrary_publish_supported === false,
      failures,
      `runtime_dataflow_subscribe must not support arbitrary publish for ${topic}`,
    )
    ensure(
      Array.isArray(subscription.body?.publishes) && subscription.body.publishes.length === 0,
      failures,
      `runtime_dataflow_subscribe must not publish for ${topic}`,
    )
    ensure(subscription.body?.transport === 'gateway_sse', failures, `runtime_dataflow_subscribe should use Gateway SSE for ${topic}`)
    ensure(subscription.body?.topic === topic, failures, `runtime_dataflow_subscribe canonical topic mismatch for ${topic}`)
    ensure(
      typeof subscription.body?.stream_url === 'string'
        && subscription.body.stream_url.includes(`/api/v1/events?topic=${encodeURIComponent(topic)}`),
      failures,
      `runtime_dataflow_subscribe stream_url should target filtered Gateway SSE for ${topic}`,
    )
    ensure(
      Array.isArray(subscription.body?.event_types) && subscription.body.event_types.length > 0,
      failures,
      `runtime_dataflow_subscribe missing event_types for ${topic}`,
    )
    ensure(
      Array.isArray(subscription.body?.stream_interfaces)
        && subscription.body.stream_interfaces.every(item => item?.transport === 'gateway_sse' && typeof item?.event_type === 'string'),
      failures,
      `runtime_dataflow_subscribe stream_interfaces should be Gateway SSE entries for ${topic}`,
    )
    const sseEvent = await readFirstSseEvent(subscription.body.stream_url)
    ensure(sseEvent.status === 200, failures, `GET ${subscription.body.stream_url} SSE expected 200 for ${topic}, got ${sseEvent.status}`)
    ensure(
      sseEvent.body?.type === 'runtime_dataflow_subscription',
      failures,
      `filtered SSE first event should be runtime_dataflow_subscription for ${topic}`,
    )
    ensure(sseEvent.body?.data?.ok === true, failures, `filtered SSE subscription event should be ok for ${topic}`)
  }

  const fieldCheckPath = links.field_check || '/api/v1/diagnostics/field-check'
  const fieldCheck = await request(fieldCheckPath, {
    method: 'POST',
    body: { mode: 'simulation' },
  })
  ensure(fieldCheck.status === 200, failures, `POST ${fieldCheckPath} expected 200, got ${fieldCheck.status}`)
  ensure(
    fieldCheck.body?.schema_version === 'lingtu.product_field_check.v1',
    failures,
    'field_check missing canonical schema version',
  )
  ensure(typeof fieldCheck.body?.ok === 'boolean', failures, 'field_check missing boolean ok')
  ensure(typeof fieldCheck.body?.runtime === 'object', failures, 'field_check missing runtime verdicts')
  ensure(typeof fieldCheck.body?.navigation === 'object', failures, 'field_check missing navigation verdicts')
  ensure(typeof fieldCheck.body?.evidence === 'object', failures, 'field_check missing evidence summary')

  const acceptancePath = links.inspection_acceptance || '/api/v1/inspection/acceptance'
  const acceptance = await request(acceptancePath, {
    method: 'POST',
    body: { mode: 'non_motion', client_id: 'web-smoke' },
  })
  ensure(acceptance.status === 200, failures, `POST ${acceptancePath} expected 200, got ${acceptance.status}`)
  ensure(
    acceptance.body?.schema_version === 'lingtu.inspection_acceptance.v1',
    failures,
    'inspection_acceptance missing canonical schema version',
  )
  ensure(typeof acceptance.body?.summary === 'string', failures, 'inspection_acceptance missing summary')
  ensure(Array.isArray(acceptance.body?.targets), failures, 'inspection_acceptance missing targets array')
  ensure(
    acceptance.body?.targets?.every(target => (
      typeof target?.status === 'string'
      && typeof target?.preview_feasible === 'boolean'
      && (target?.preview_count == null || typeof target.preview_count === 'number')
      && Array.isArray(target?.reasons)
    )),
    failures,
    'inspection_acceptance targets must expose preview status, feasibility, count, and reasons',
  )

  const ready = await request('/ready', { json: false })
  if (!allowDegradedReady) {
    ensure(ready.status === 200, failures, `GET /ready expected 200, got ${ready.status}`)
  } else {
    ensure([200, 503].includes(ready.status), failures, `GET /ready expected 200 or 503, got ${ready.status}`)
  }

  const session = bootstrapBody.session || {}
  const localization = bootstrapBody.localization || {}
  const navigation = bootstrapBody.navigation || {}

  const summary = {
    base,
    ready: ready.status,
    session_mode: session.mode ?? null,
    active_map: session.active_map ?? null,
    localization_state: localization.state ?? null,
    localization_backend: session.localization_backend ?? session.slam_profile ?? null,
    can_accept_goal: navigation.can_accept_goal ?? navigation.readiness?.can_accept_goal ?? null,
    routecheck_ok: typeof routecheck.body?.ok === 'boolean' ? routecheck.body.ok : null,
    routecheck_count: typeof routecheck.body?.count === 'number' ? routecheck.body.count : null,
    real_runtime_evidence_ok: typeof realEvidence.body?.ok === 'boolean' ? realEvidence.body.ok : null,
    dataflow_topics: Array.isArray(dataflow.body?.topics) ? dataflow.body.topics.length : null,
    dataflow_live: Array.isArray(dataflow.body?.topics)
      ? dataflow.body.topics.filter(topic => topic?.inspection?.live === true).length
      : null,
    dataflow_stages: Array.isArray(dataflow.body?.stage_evidence) ? dataflow.body.stage_evidence.length : null,
    inspection_summary: typeof acceptance.body?.summary === 'string' ? acceptance.body.summary : null,
    inspection_targets: Array.isArray(acceptance.body?.targets) ? acceptance.body.targets.length : null,
    link_count: Object.keys(links).length,
  }

  console.log(JSON.stringify({ ok: failures.length === 0, summary, failures }, null, 2))
} catch (err) {
  failures.push(err instanceof Error ? err.message : String(err))
  console.log(JSON.stringify({ ok: false, base, failures }, null, 2))
}

if (failures.length > 0) {
  process.exitCode = 1
}
