import assert from 'node:assert/strict'
import test from 'node:test'
import { Children, createElement, isValidElement, type ReactElement, type ReactNode } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import { createServer } from 'vite'
import {
  fetchRecordingCatalog,
  fetchRecordingSummary,
  recordingArchiveUrl,
  startRecording,
  validateRecording,
} from '../src/services/api.ts'
import type { RecordingTopic } from '../src/types/index.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

const topics: RecordingTopic[] = [
  { topic: '/imu/raw', type_name: 'Imu', group: 'sensors', label: '原始 IMU', replayable: true, default_selected: true },
  { topic: '/driver/joint_state', type_name: 'JointState', group: 'robot', label: '关节与电机状态', replayable: false, default_selected: false },
  { topic: '/driver/control_state', type_name: 'ControlState', group: 'robot', label: '底盘控制状态', replayable: false, default_selected: true },
]

test('explicit UI stream selection reaches the recorder without reverting to the sensor preset', async () => {
  let sent: unknown
  globalThis.fetch = async (url, options) => {
    assert.equal(url, '/api/v1/recordings/start')
    sent = JSON.parse(String(options?.body))
    return Response.json({ state: 'recording' })
  }
  await startRecording({ topics: ['/driver/joint_state', '/slam/odometry'], camera: true, duration: 120 })
  assert.deepEqual(sent, {
    duration: 120, prefix: 'web', capture_profile: 'sensors', camera: true,
    minimum_free_gib: 5, topics: ['/driver/joint_state', '/slam/odometry'],
  })
})

test('camera-only selection explicitly sends no DDS streams', async () => {
  let sent: Record<string, unknown> | undefined
  globalThis.fetch = async (_url, options) => {
    sent = JSON.parse(String(options?.body)) as Record<string, unknown>
    return Response.json({ state: 'recording' })
  }
  await startRecording({ topics: [], camera: true })
  assert.deepEqual(sent?.topics, [])
  assert.equal(sent?.camera, true)
})

test('catalog, summary, archive and decode validation use the selected session without starting timed replay', async () => {
  const requests: Array<[string, string]> = []
  globalThis.fetch = async (url, options) => {
    requests.push([String(url), options?.method ?? 'GET'])
    return Response.json({ ok: true })
  }
  await fetchRecordingCatalog()
  await fetchRecordingSummary('test recording')
  await validateRecording('test recording')
  assert.equal(recordingArchiveUrl('test recording'), '/api/v1/recordings/test%20recording/archive')
  assert.deepEqual(requests, [
    ['/api/v1/recordings/catalog', 'GET'],
    ['/api/v1/recordings/test%20recording/summary', 'GET'],
    ['/api/v1/recordings/test%20recording/validate', 'POST'],
  ])
})

interface ElementProps {
  children?: ReactNode
  type?: string
  checked?: boolean
  disabled?: boolean
  onClick?: () => void
  onChange?: (event: { target: { checked: boolean } }) => void
}

function descendants(node: ReactNode): Array<ReactElement<ElementProps>> {
  return Children.toArray(node).flatMap(child => isValidElement<ElementProps>(child)
    ? [child, ...descendants(child.props.children)] : [])
}

test('topic picker toggles actual catalog entries and never invents motor streams', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingTopicPicker, RecordingReceivedStreams } = await server.ssrLoadModule('/src/components/RecordingPanel.tsx')
    let selected = ['/imu/raw']
    const render = (disabled = false) => RecordingTopicPicker({
      topics, selected, disabled, locale: 'zh', onChange: (next: string[]) => { selected = next },
    }) as ReactElement
    const markup = renderToStaticMarkup(render())
    assert.match(markup, /关节与电机状态/)
    assert.match(markup, /机器人状态/)
    assert.match(markup, /仅分析/)
    const checkboxes = descendants(render()).filter(item => item.type === 'input' && item.props.type === 'checkbox')
    assert.equal(checkboxes.length, topics.length)
    checkboxes[1].props.onChange?.({ target: { checked: true } })
    assert.deepEqual(selected, ['/imu/raw', '/driver/joint_state'])
    const buttons = descendants(render()).filter(item => item.type === 'button')
    buttons[1].props.onClick?.()
    assert.deepEqual(selected, topics.map(topic => topic.topic))
    descendants(render()).filter(item => item.type === 'button')[0].props.onClick?.()
    assert.deepEqual(selected, ['/imu/raw', '/driver/control_state'])
    descendants(render()).filter(item => item.type === 'button')[2].props.onClick?.()
    assert.deepEqual(selected, [])
    assert.ok(descendants(render(true)).filter(item => item.type === 'input' || item.type === 'button').every(item => item.props.disabled))
    const samples = renderToStaticMarkup(createElement(RecordingReceivedStreams, {
      topics, locale: 'zh', summary: {
        topics: [{ topic: '/imu/raw', dds_topic: 'rt/imu/raw', type_name: 'Imu', message_count: 8, replayable: true }],
        missing_topics: ['/driver/joint_state'],
      },
    }))
    assert.match(samples, /原始 IMU/)
    assert.match(samples, /关节与电机状态/)
    assert.match(samples, /未收到/)
    assert.match(samples, /title="rt\/imu\/raw"/)
  } finally {
    await server.close()
  }
})
