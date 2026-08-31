import { useCallback, useEffect, useMemo, useState } from 'react'
import { Radio, RotateCcw } from 'lucide-react'
import type {
  ProductFieldCheckResponse,
  RuntimeDataflowResponse,
  RuntimeDataflowStageEvidence,
  RuntimeDataflowSubscribeResponse,
  RuntimeDataflowTopicDetailResponse,
  RuntimeDataflowTopicSummary,
  SSEState,
} from '../types'
import * as api from '../services/api'
import styles from './RuntimeDataflowView.module.css'

interface RuntimeDataflowViewProps {
  sseState: SSEState
}

function asBool(value: unknown): boolean {
  return value === true
}

function asRecord(value: unknown): Record<string, unknown> {
  return typeof value === 'object' && value !== null ? value as Record<string, unknown> : {}
}

function stringValue(value: unknown, fallback = '--'): string {
  return typeof value === 'string' && value.length > 0 ? value : fallback
}

function numberValue(value: unknown, digits = 1): string {
  return typeof value === 'number' && Number.isFinite(value)
    ? value.toFixed(digits)
    : '--'
}

function topicLabel(topic: string): string {
  return topic.split('/').filter(Boolean).pop() ?? topic
}

function itemPath(item: Record<string, unknown>): string {
  return stringValue(item.path ?? item.event_type ?? item.payload ?? item.field, 'declared')
}

function itemTransport(item: Record<string, unknown>): string {
  return stringValue(item.transport ?? item.method ?? item.type, 'gateway')
}

function compactTokens(tokens: string[], limit = 3): string {
  if (tokens.length === 0) return '--'
  const visible = tokens.slice(0, limit).join(', ')
  return tokens.length > limit ? `${visible} +${tokens.length - limit}` : visible
}

function statusOf(topic: RuntimeDataflowTopicSummary): {
  label: string
  tone: 'live' | 'meta' | 'none'
} {
  if (topic.inspection.live) return { label: 'LIVE', tone: 'live' }
  if (topic.inspection.observation_level === 'metadata_only') {
    return { label: 'META', tone: 'meta' }
  }
  return { label: 'NO SAMPLE', tone: 'none' }
}

function stageTone(stage: RuntimeDataflowStageEvidence): 'live' | 'meta' | 'none' {
  if (stage.live) return 'live'
  if (stage.observable) return 'meta'
  return 'none'
}

function stageIssue(stage: RuntimeDataflowStageEvidence): string {
  const missing = [...stage.missing_inputs, ...stage.missing_outputs]
  if (missing.length > 0) return `missing ${compactTokens(missing)}`
  const notLive = [...stage.not_live_inputs, ...stage.not_live_outputs]
  if (notLive.length > 0) return `not live ${compactTokens(notLive)}`
  return stage.live ? 'fresh module sample' : 'metadata only'
}

function commandPolicyText(dataflow: RuntimeDataflowResponse | null): string {
  const policy = dataflow?.control_boundary?.policy
  return typeof policy === 'string' && policy.length > 0
    ? policy
    : 'whitelisted_gateway_commands_only'
}

function isRealRuntimeBoundary(dataflow: RuntimeDataflowResponse): boolean {
  const boundary = asRecord(dataflow.runtime_boundary)
  return boundary.env === 'real'
}

export function RuntimeDataflowView({ sseState }: RuntimeDataflowViewProps) {
  const [dataflow, setDataflow] = useState<RuntimeDataflowResponse | null>(null)
  const [fieldCheck, setFieldCheck] = useState<ProductFieldCheckResponse | null>(null)
  const [subscriptionPlan, setSubscriptionPlan] = useState<RuntimeDataflowSubscribeResponse | null>(null)
  const [streamEvent, setStreamEvent] = useState<Record<string, unknown> | null>(null)
  const [detail, setDetail] = useState<RuntimeDataflowTopicDetailResponse | null>(null)
  const [selected, setSelected] = useState<string | null>(null)
  const [loading, setLoading] = useState(false)
  const [detailLoading, setDetailLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [detailError, setDetailError] = useState<string | null>(null)
  const [fieldError, setFieldError] = useState<string | null>(null)
  const [streamError, setStreamError] = useState<string | null>(null)

  const loadSummary = useCallback(async () => {
    setLoading(true)
    try {
      const next = await api.fetchRuntimeDataflow()
      const realRuntime = isRealRuntimeBoundary(next)
      const fieldCheckMode = realRuntime ? 'field' : 'simulation'
      const [fieldCheckResult] = await Promise.allSettled([
        api.runProductFieldCheck({ mode: fieldCheckMode }),
      ])
      setDataflow(next)
      setError(null)
      setSelected(prev => prev ?? next.topics[0]?.topic ?? null)
      setFieldCheck(fieldCheckResult.status === 'fulfilled' ? fieldCheckResult.value : null)
      setFieldError(
        fieldCheckResult.status === 'rejected'
          ? fieldCheckResult.reason instanceof Error
            ? fieldCheckResult.reason.message
            : String(fieldCheckResult.reason)
          : null,
      )
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }, [])

  const loadDetail = useCallback(async (topic: string | null) => {
    if (!topic) {
      setDetail(null)
      return
    }
    setDetailLoading(true)
    try {
      const next = await api.fetchRuntimeDataflowTopic(topic)
      setDetail(next)
      setDetailError(next.ok ? null : next.error ?? 'runtime_topic_not_found')
    } catch (err) {
      setDetailError(err instanceof Error ? err.message : String(err))
    } finally {
      setDetailLoading(false)
    }
  }, [])

  useEffect(() => {
    void loadSummary()
    const timer = window.setInterval(() => {
      void loadSummary()
    }, 5000)
    return () => window.clearInterval(timer)
  }, [loadSummary])

  useEffect(() => {
    void loadDetail(selected)
  }, [loadDetail, selected])

  useEffect(() => {
    if (!selected) {
      setSubscriptionPlan(null)
      setStreamEvent(null)
      setStreamError(null)
      return
    }

    let cancelled = false
    let source: EventSource | null = null

    setStreamError(null)
    setStreamEvent(null)
    void api.subscribeRuntimeDataflow({ selector: selected }).then(subscriptionPlan => {
      if (cancelled) return
      setSubscriptionPlan(subscriptionPlan)
      if (!subscriptionPlan.ok || !subscriptionPlan.stream_url) {
        setStreamError(subscriptionPlan.blockers[0] ?? 'no_gateway_sse_stream')
        return
      }

      const recordEvent = (event: MessageEvent, eventType: string) => {
        try {
          const raw = JSON.parse(event.data) as unknown
          const payload = asRecord(raw)
          setStreamEvent({
            ...payload,
            event_type: eventType,
            received_at: Date.now(),
          })
        } catch {
          setStreamEvent({
            event_type: eventType,
            received_at: Date.now(),
            data: event.data,
          })
        }
      }

      source = new EventSource(subscriptionPlan.stream_url)
      source.onmessage = event => recordEvent(event, 'message')
      for (const eventType of subscriptionPlan.event_types) {
        source.addEventListener(eventType, event => {
          recordEvent(event as MessageEvent, eventType)
        })
      }
      source.onerror = () => {
        if (!cancelled) setStreamError('stream disconnected or unavailable')
      }
    }).catch(err => {
      if (!cancelled) {
        setSubscriptionPlan(null)
        setStreamError(err instanceof Error ? err.message : String(err))
      }
    })

    return () => {
      cancelled = true
      source?.close()
    }
  }, [selected])

  const topics = useMemo(() => dataflow?.topics ?? [], [dataflow?.topics])
  const stageEvidence = useMemo(() => dataflow?.stage_evidence ?? [], [dataflow?.stage_evidence])
  const liveCount = topics.filter(topic => topic.inspection.live).length
  const observableCount = topics.filter(topic => topic.inspection.observable).length
  const selectedTopic = useMemo(
    () => topics.find(topic => topic.topic === selected) ?? detail?.topic ?? null,
    [detail?.topic, selected, topics],
  )
  const selectedInspection = detail?.inspection ?? selectedTopic?.inspection ?? {}
  const eventTypes = subscriptionPlan?.event_types ?? []
  const moduleStats = selectedInspection.module_stats ?? []
  const payloadInterfaces = selectedInspection.payload_interfaces ?? []
  const writeInterfaces = selectedInspection.write_interfaces ?? []
  const latestPayload = selectedInspection.latest_payload
  const policy = commandPolicyText(dataflow)
  const arbitraryPublish = asBool(dataflow?.control_boundary?.arbitrary_publish_supported)
  const modulePortPrimary = asBool(dataflow?.transport_layers?.module_port_bus?.primary)
  const fieldRuntime = asRecord(fieldCheck?.runtime)
  const fieldNavigation = asRecord(fieldCheck?.navigation)
  const fieldEvidence = asRecord(fieldCheck?.evidence)
  const fieldReady = fieldCheck?.ok === true
  const topBlocker = fieldError ?? fieldCheck?.blockers?.[0] ?? (
    fieldReady ? 'Gateway product evidence is passing' : 'Product check unavailable'
  )
  const traffic = sseState.traffic

  return (
    <div className={styles.page} role="tabpanel" id="panel-dataflow">
      <header className={styles.header}>
        <div>
          <div className={styles.eyebrow}>Runtime Dataflow</div>
          <h1 className={styles.title}>Gateway + ModulePorts</h1>
        </div>
        <button
          type="button"
          className={styles.refreshButton}
          onClick={() => {
            void loadSummary()
            void loadDetail(selected)
          }}
          disabled={loading || detailLoading}
          title="Refresh runtime dataflow"
        >
          <RotateCcw size={14} />
          Refresh
        </button>
      </header>

      <section className={styles.fieldReadyStrip} aria-label="product runtime check summary">
        <div>
          <div className={styles.eyebrow}>Product Check</div>
          <div className={fieldReady ? styles.fieldReadyStatusPass : styles.fieldReadyStatusFail}>
            {fieldReady ? 'PASS' : 'FAIL'}
          </div>
        </div>
        <div className={styles.fieldReadyChecks}>
          <span>Gateway {stringValue(fieldRuntime.gateway)}</span>
          <span>Readiness {stringValue(fieldRuntime.readiness)}</span>
          <span>Localization {stringValue(fieldRuntime.localization)}</span>
          <span>Observability {stringValue(fieldRuntime.gateway_observability)}</span>
          <span>Goal {stringValue(fieldNavigation.can_send_goal)}</span>
          <span>Driver Command {stringValue(fieldNavigation.driver_command)}</span>
          <span>Field Evidence {stringValue(fieldEvidence.field_runtime)}</span>
        </div>
        <div className={styles.fieldReadyBlocker} title={topBlocker}>
          {topBlocker}
        </div>
      </section>

      <section className={styles.summaryGrid} aria-label="runtime dataflow summary">
        <Metric label="Contract" value={stringValue(dataflow?.runtime_contract)} />
        <Metric
          label="ModulePorts"
          value={modulePortPrimary ? 'PRIMARY' : 'UNKNOWN'}
          tone={modulePortPrimary ? 'ok' : 'warn'}
        />
        <Metric label="Live Streams" value={`${liveCount}/${topics.length}`} tone={liveCount > 0 ? 'ok' : 'warn'} />
        <Metric label="Observable" value={`${observableCount}/${topics.length}`} />
        <Metric label="Stage Evidence" value={`${stageEvidence.length}`} tone={stageEvidence.length > 0 ? 'ok' : 'warn'} />
        <Metric label="Command Policy" value={policy} />
        <Metric
          label="Arbitrary Publish"
          value={arbitraryPublish ? 'SUPPORTED' : 'FALSE'}
          tone={arbitraryPublish ? 'warn' : 'ok'}
        />
      </section>

      {error && <div className={styles.error}>Runtime dataflow unavailable: {error}</div>}

      <div className={styles.content}>
        <section className={styles.tablePanel} aria-label="runtime stream table">
          <div className={styles.panelTitleRow}>
            <div>
              <div className={styles.sectionTitle}>Product Streams</div>
              <div className={styles.caption}>Read-only Gateway inspection</div>
            </div>
            <span className={styles.pollPill}>{loading ? 'LOADING' : '5s POLL'}</span>
          </div>

          <div className={styles.table}>
            {topics.map(topic => {
              const status = statusOf(topic)
              const selectedRow = selected === topic.topic
              const port = topic.inspection.module_stats?.[0]
              const channels = topic.inspection.payload_interfaces ?? []
              return (
                <button
                  type="button"
                  key={topic.topic}
                  className={selectedRow ? styles.rowSelected : styles.row}
                  onClick={() => setSelected(topic.topic)}
                >
                  <span className={styles.streamName}>
                    <span className={`${styles.statusDot} ${styles[status.tone]}`} />
                    <span>
                      <span className={styles.streamLabel}>{topicLabel(topic.topic)}</span>
                      <span className={styles.streamTopic}>{topic.topic}</span>
                    </span>
                  </span>
                  <span className={styles.rowCell}>{status.label}</span>
                  <span className={styles.rowCell}>{port ? `${port.module}.${port.port}` : 'no module sample'}</span>
                  <span className={styles.rowCell}>{port ? `${port.msg_count} msg / ${numberValue(port.rate_hz)} Hz` : '--'}</span>
                  <span className={styles.rowCell}>{channels.length ? channels.map(itemTransport).join(', ') : 'metadata only'}</span>
                  <span className={topic.communication.allowed ? styles.rowWarn : styles.rowDim}>
                    {topic.communication.allowed ? 'WHITELIST CMD' : 'READ ONLY'}
                  </span>
                </button>
              )
            })}
          </div>

          <section className={styles.stageSection} aria-label="runtime stage evidence">
            <div className={styles.panelTitleRow}>
              <div>
                <div className={styles.sectionTitle}>Stage Evidence</div>
                <div className={styles.caption}>Expected inputs and outputs from the runtime contract</div>
              </div>
              <span className={styles.pollPill}>READ ONLY</span>
            </div>
            <div className={styles.stageTable}>
              {stageEvidence.map(stage => {
                const tone = stageTone(stage)
                return (
                  <div className={styles.stageRow} key={stage.name}>
                    <span className={styles.streamName}>
                      <span className={`${styles.statusDot} ${styles[tone]}`} />
                      <span>
                        <span className={styles.streamLabel}>{stage.name}</span>
                        <span className={styles.streamTopic}>{stage.owner ?? 'runtime stage'}</span>
                      </span>
                    </span>
                    <span className={styles.rowCell}>{stage.status.toUpperCase()}</span>
                    <span className={styles.rowCell} title={stage.inputs.join(', ')}>
                      in {compactTokens(stage.inputs)}
                    </span>
                    <span className={styles.rowCell} title={stage.outputs.join(', ')}>
                      out {compactTokens(stage.outputs)}
                    </span>
                    <span className={stage.live ? styles.rowDim : styles.rowWarn} title={stageIssue(stage)}>
                      {stageIssue(stage)}
                    </span>
                  </div>
                )
              })}
              {stageEvidence.length === 0 && (
                <div className={styles.stageRow}>
                  <span className={styles.rowWarn}>No stage_evidence in runtime dataflow response</span>
                </div>
              )}
            </div>
          </section>
        </section>

        <aside className={styles.detailPanel} aria-label="runtime stream detail">
          <div className={styles.panelTitleRow}>
            <div>
              <div className={styles.sectionTitle}>Stream Detail</div>
              <div className={styles.caption}>{selected ?? 'Select a stream'}</div>
            </div>
            <span className={detail?.ok === false ? styles.failPill : styles.passPill}>
              {detailLoading ? 'LOADING' : detail?.ok === false ? 'NOT FOUND' : 'DETAIL'}
            </span>
          </div>

          {detailError && <div className={styles.error}>Detail: {detailError}</div>}

          <section className={styles.detailSection}>
            <div className={styles.sectionTitle}>Observation</div>
            <div className={styles.detailGrid}>
              <Metric label="Level" value={stringValue(selectedInspection.observation_level)} />
              <Metric
                label="Live"
                value={selectedInspection.live ? 'TRUE' : 'FALSE'}
                tone={selectedInspection.live ? 'ok' : 'warn'}
              />
              <Metric
                label="Payload"
                value={selectedInspection.payload_available ? 'AVAILABLE' : 'METADATA'}
                tone={selectedInspection.payload_available ? 'ok' : 'dim'}
              />
            </div>
          </section>

          <section className={styles.detailSection}>
            <div className={styles.sectionTitle}>Realtime Stream</div>
            <div className={styles.detailGrid}>
              <Metric
                label="Plan"
                value={subscriptionPlan?.ok ? 'SUBSCRIBED' : subscriptionPlan ? 'BLOCKED' : 'CHECK'}
                tone={subscriptionPlan?.ok ? 'ok' : 'warn'}
              />
              <Metric label="event_types" value={compactTokens(eventTypes)} />
              <Metric
                label="Read Only"
                value={subscriptionPlan?.read_only === true ? 'TRUE' : 'CHECK'}
                tone={subscriptionPlan?.read_only === true ? 'ok' : 'warn'}
              />
              <Metric
                label="Publishes"
                value={subscriptionPlan?.publishes?.length ? subscriptionPlan.publishes.join(', ') : 'none'}
                tone={subscriptionPlan?.publishes?.length ? 'warn' : 'ok'}
              />
            </div>
            {streamError && <div className={styles.error}>Realtime stream: {streamError}</div>}
            <pre className={styles.payloadPreview}>
              {streamEvent === null
                ? 'Waiting for selected Gateway SSE stream'
                : JSON.stringify(streamEvent, null, 2)}
            </pre>
          </section>

          <section className={styles.detailSection}>
            <div className={styles.sectionTitle}>ModulePort Samples</div>
            <ul className={styles.list}>
              {moduleStats.length > 0 ? moduleStats.map(port => (
                <li key={`${port.module}-${port.port}-${port.direction}`}>
                  {port.module}.{port.port} {port.direction} - {port.msg_count} msg / {numberValue(port.rate_hz)} Hz / stale {numberValue(port.stale_ms, 0)} ms
                </li>
              )) : <li>No fresh ModulePort sample yet</li>}
            </ul>
          </section>

          <section className={styles.detailSection}>
            <div className={styles.sectionTitle}>Gateway Payload</div>
            <ul className={styles.list}>
              {payloadInterfaces.length > 0 ? payloadInterfaces.map(item => (
                <li key={`${itemTransport(item)}-${itemPath(item)}`}>
                  {itemTransport(item)}: {itemPath(item)}
                </li>
              )) : <li>No declared Gateway payload channel</li>}
            </ul>
          </section>

          <section className={styles.detailSection}>
            <div className={styles.sectionTitle}>Latest Payload</div>
            <pre className={styles.payloadPreview}>
              {latestPayload === undefined || latestPayload === null
                ? 'No payload sample'
                : JSON.stringify(latestPayload, null, 2)}
            </pre>
          </section>

          <section className={styles.detailSection}>
            <div className={styles.sectionTitle}>Communication Boundary</div>
            <ul className={styles.list}>
              <li>arbitrary_publish_supported={String(Boolean(selectedInspection.arbitrary_publish_supported))}</li>
              <li>policy={stringValue(selectedInspection.policy, policy)}</li>
              {writeInterfaces.length > 0
                ? writeInterfaces.map(item => (
                  <li key={`${itemTransport(item)}-${itemPath(item)}`}>
                    {itemTransport(item)}: {itemPath(item)}
                  </li>
                ))
                : <li>read-only observation</li>}
            </ul>
          </section>
        </aside>
      </div>

      <section className={styles.healthStrip} aria-label="realtime health">
        <span><Radio size={13} /> SSE {sseState.connected ? 'connected' : 'offline'}</span>
        <span>event_id {sseState.lastEventId ?? '--'}</span>
        <span>missed {sseState.missedEvents}</span>
        <span>reconnects {sseState.reconnects}</span>
        <span>traffic {traffic?.status ?? 'pending'}</span>
        <span>sse drops {traffic?.sse.dropped_events ?? '--'}</span>
        <span>cloud drops {traffic?.cloud.dropped_frames ?? '--'}</span>
      </section>
    </div>
  )
}

function Metric({
  label,
  value,
  tone = 'dim',
}: {
  label: string
  value: string
  tone?: 'ok' | 'warn' | 'dim'
}) {
  const valueClass =
    tone === 'ok' ? styles.metricOk :
    tone === 'warn' ? styles.metricWarn :
    styles.metricValue
  return (
    <div className={styles.metric}>
      <span className={styles.metricLabel}>{label}</span>
      <span className={valueClass} title={value}>{value}</span>
    </div>
  )
}
