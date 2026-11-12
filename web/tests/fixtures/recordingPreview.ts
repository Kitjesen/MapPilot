import type { RecordingCatalogResponse, RecordingListResponse, RecordingStatusResponse, RecordingStreamsResponse } from '../../src/types/index.ts'

const streams = [
  { topic: '/lidar/points', label: '激光点云', type_name: 'PointCloud', group: 'sensors', rate: 10, bandwidth: 5_242_880, preview: { point_count: 24000, frame_id: 'lidar', scan_duration_s: 0.1 } },
  { topic: '/imu/raw', label: '惯性测量', type_name: 'Imu', group: 'sensors', rate: 200, bandwidth: 209_715, preview: { angular_velocity: { x: 0.02, y: -0.01, z: 0.12 }, linear_acceleration: { x: 0.1, y: 0.04, z: 9.81 } } },
  { topic: '/slam/odometry', label: '定位里程计', type_name: 'Odometry', group: 'localization', rate: 50, bandwidth: 838_861, preview: { position: { x: 2.4, y: 1.2, z: 0.46 }, velocity: { x: 0.5, y: 0, z: 0 } } },
  { topic: '/slam/map', label: '局部地图', type_name: 'PointCloud', group: 'localization', rate: 2, bandwidth: 671_089, preview: { point_count: 84000, resolution_m: 0.1, frame_id: 'map' } },
  { topic: '/robot/joint_states', label: '电机关节', type_name: 'JointState', group: 'robot', rate: 100, bandwidth: 503_316, preview: { position: [0.1, 0.8, -1.5, -0.1, 0.8, -1.5], velocity: [0.2, -0.1, 0.3], temperature_c: [36, 37, 38] } },
  { topic: '/robot/state', label: '机身状态', type_name: 'RobotState', group: 'robot', rate: 50, bandwidth: 251_658, preview: { battery_percent: 82, voltage_v: 25.4, body_height_m: 0.46 } },
  { topic: '/nav/path', label: '规划路径', type_name: 'Path', group: 'navigation', rate: 10, bandwidth: 503_316, preview: { waypoint_count: 42, remaining_distance_m: 8.6, target: { x: 10, y: 4, z: 0.46 } } },
  { topic: '/inspection/events', label: '巡检事件', type_name: 'InspectionEvent', group: 'inspection', rate: 2, bandwidth: 167_773, preview: { checkpoint: 'A-03', temperature_c: 32.5, confidence: 0.98 } },
]

export const recordingPreviewCatalog: RecordingCatalogResponse = {
  ok: true,
  version: 1,
  topics: streams.map(({ topic, label, type_name, group }) => ({ topic, label, type_name, group, replayable: true, default_selected: true })),
  presets: [],
  camera: { supported: false, label: '演示页未接入相机' },
}

const demoSessions: RecordingListResponse = {
  ok: true, truncated: false, disk_free: 128 * 1024 ** 3, disk_total: 256 * 1024 ** 3,
  sessions: ['园区巡检', '定位观测', '传感器检查'].map((name, index) => ({ session_id: `demo-${index + 1}-${name}（演示）`, state: '演示记录' })),
}

// This fetch exists only in the test entry. Unknown requests never reach a Gateway.
export function createRecordingPreviewFetch(now: () => number = Date.now): typeof fetch {
  const startedAt = now()
  return async (input, options) => {
    const request = input instanceof Request ? input : null
    const url = new URL(request?.url ?? String(input), 'http://localhost')
    const method = options?.method ?? request?.method ?? 'GET'
    if (method !== 'GET') return Response.json({ error: '演示页仅供查看，无法操作录制或机器人。' }, { status: 405 })
    const timestamp = now()
    if (url.pathname === '/api/v1/recordings/catalog') return Response.json(recordingPreviewCatalog)
    if (url.pathname === '/api/v1/recordings/status') {
      const status: RecordingStatusResponse = {
        available: true, healthy: true, backend: 'demo', state: 'idle', recording: false,
        duration_s: 0, size_bytes: 0, disk_free: 128 * 1024 ** 3, disk_total: 256 * 1024 ** 3,
      }
      return Response.json(status)
    }
    if (url.pathname === '/api/v1/recordings/streams') {
      const elapsed = 60 + Math.max(0, timestamp - startedAt) / 1000
      const snapshot: RecordingStreamsResponse = {
        ok: true, available: true, state: 'observing', snapshot_age_ms: 0,
        observed_at_unix_ns: timestamp * 1e6, window_s: 1,
        topics: streams.map(stream => ({
          topic: stream.topic, type_name: stream.type_name,
          message_count: Math.floor(elapsed * stream.rate),
          bytes_received: Math.floor(elapsed * stream.bandwidth),
          rate_hz: stream.rate, bytes_per_second: stream.bandwidth,
          last_sample_age_ms: 15, source_time_ns: (timestamp - 15) * 1e6,
          preview: { ...stream.preview, demo_signal: Number(Math.sin(elapsed / 4).toFixed(3)) },
        })),
      }
      return Response.json(snapshot)
    }
    if (url.pathname === '/api/v1/recordings') return Response.json(demoSessions)
    const pathname = decodeURI(url.pathname)
    const session = demoSessions.sessions.find(item => pathname === `/api/v1/recordings/${item.session_id}`
      || pathname.startsWith(`/api/v1/recordings/${item.session_id}/`))
    if (session) {
      const path = pathname.slice(`/api/v1/recordings/${session.session_id}`.length)
      if (path === '') return Response.json({ ok: true, session: {
        ...session, context: { demo: true }, artifacts: [],
        children: [{ name: 'dds', artifacts: [], selected_topics: ['/slam/odometry'] }],
      } })
      if (path === '/summary') return Response.json({ ok: true, summary: {
        message_count: 3, start_time_ns: 0, end_time_ns: 2e9, duration_ns: 2e9, missing_topics: [],
        topics: [{ topic: '/slam/odometry', dds_topic: '/slam/odometry', type_name: 'Odometry', message_count: 3, replayable: false }],
      } })
      if (path === '/series' && url.searchParams.get('topic') === '/slam/odometry') return Response.json({ ok: true, session_id: session.session_id, series: {
        version: 1, ok: true, mode: 'series', topic: '/slam/odometry', field: 'position.x',
        fields: [{ path: 'position.x', unit: 'm' }], start_time_ns: '0', duration_s: 2,
        window: { start_s: 0, end_s: 2 }, samples: [{ t: 0, value: 0 }, { t: 1, value: 0.5 }, { t: 2, value: 1 }],
        original_count: 3, read_count: 3, numeric_count: 3, returned_count: 3,
        downsampling: { method: 'none', max_points: 2000 }, frame_id: 'demo',
      } })
      if (path === '/scene-info') return Response.json({ ok: true, session_id: session.session_id, state: session.state,
        scene: null, start_time_ns: '0', duration_s: 2,
        camera: { available: false, timing: 'frame_timestamp_index', exact: false, segments: [] },
      })
    }
    if (url.pathname === '/api/v1/camera/status') return Response.json({ status: 'not_loaded' })
    return Response.json({ error: '演示页没有此数据。' }, { status: 404 })
  }
}
