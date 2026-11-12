import { useEffect, useState } from 'react'
import { createRoot } from 'react-dom/client'
import { SceneView } from '../src/components/SceneView'
import type { SSEState } from '../src/types'
import plan from '../../artifacts/deploy/go2-20261109/nav-903/nav-903-plan.json'
import navigation from '../../artifacts/deploy/go2-20261109/nav-903/nav-903-navigation.json'
import request from '../../artifacts/deploy/go2-20261109/nav-903/nav-903-headless-request.json'
import '../src/App.css'

const map = '903room_v6_corridor'
const sessionId = 'offline-903-ui-check'
const fetchLocal = window.fetch.bind(window)
const json = (value: unknown) => new Response(JSON.stringify(value), { headers: { 'Content-Type': 'application/json' } })
window.fetch = async (input, init) => {
  const path = new URL(String(input), location.href).pathname
  if (path === '/api/v1/navigation/goal_candidate') return fetchLocal(input, init)
  if (init?.method && init.method !== 'GET') return new Response('Offline preview cannot command a robot', { status: 403 })
  if (path.endsWith('/pcd')) {
    const response = await fetchLocal('/@fs/D:/inovxio/brain/lingtu/build/nav-field-20261025/active-v6-map.pcd')
    return new Response(await response.arrayBuffer(), { headers: {
      'X-Lingtu-Map-Id': map, 'X-Lingtu-Content-Epoch': '1', 'X-Lingtu-Frame-Id': 'map',
    } })
  }
  if (path.endsWith('/points')) return json({ name: map, frame_id: 'map', epoch: 1, content_epoch: 1 })
  if (path === '/api/v1/slam/maps') return json({ maps: [{ name: map, is_active: true, content_epoch: 1, point_count: 3836924, can_activate: true }] })
  if (path.includes('dataflow')) return json({ runtime_boundary: { product_session_id: sessionId }, run_plan: plan })
  if (path.includes('/inspection/tasks')) return json({ tasks: [] })
  if (path.includes('/camera')) return json({ available: false, status: 'unavailable' })
  if (path.includes('/recording')) return json({ active: false, state: 'idle' })
  return new Response('No live telemetry in offline preview', { status: 503 })
}

export function Preview() {
  const [now, setNow] = useState(() => Date.now())
  const [notice, setNotice] = useState('')
  useEffect(() => { const id = setInterval(() => setNow(Date.now()), 500); return () => clearInterval(id) }, [])
  useEffect(() => {
    let cancelled = false
    const frame = () => new Promise<void>(resolve => requestAnimationFrame(() => resolve()))
    const ready = async (selector: string) => {
      const began = Date.now()
      while (!cancelled && Date.now() - began < 90_000) {
        const element = document.querySelector<HTMLElement>(selector)
        if (element) return element
        await new Promise(resolve => setTimeout(resolve, 200))
      }
      throw new Error(`Missing preview control: ${selector}`)
    }
    const blurValue = async (label: string, value: string) => {
      const input = await ready(`input[aria-label="${label}"]`) as HTMLInputElement
      Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value')!.set!.call(input, value)
      input.dispatchEvent(new FocusEvent('focusout', { bubbles: true }))
      await frame()
    }
    void (async () => {
      await ready('button[aria-label="查看完整保存地图"]:not(:disabled)')
      ;(await ready('button[aria-label="图层"]')).click()
      ;(await ready('button[title="固定高度范围"]')).click()
      await blurValue('显示高度下限', '-0.8')
      await blurValue('显示高度上限', '0.35')
      const size = await ready('input[aria-label="点云大小"]') as HTMLInputElement
      Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value')!.set!.call(size, '0.035')
      size.dispatchEvent(new Event('input', { bubbles: true }))
      await frame()
      ;(await ready('button[aria-label="图层"]')).click()
      ;(await ready('[aria-label="显示设置"]')).click()
      ;(await ready('button[aria-label="俯视当前数据"]')).click()
      await frame(); await frame()
      const canvas = await ready('canvas')
      const rect = canvas.getBoundingClientRect()
      const event = { bubbles: true, button: 0, clientX: rect.left + rect.width / 2, clientY: rect.top + rect.height / 2 }
      canvas.dispatchEvent(new MouseEvent('mousedown', event))
      canvas.dispatchEvent(new MouseEvent('mouseup', event))
      const card = await ready('[aria-label="导航目标"]')
      const settings = [...card.querySelectorAll('details')].find(item => item.querySelector('summary')?.textContent?.includes('调整'))!
      settings.open = true
      await blurValue('目标 X（米）', '-0.267451257321')
      await blurValue('目标 Y（米）', '4.98830053134')
      settings.open = false
      setNotice('控件回放：点选目标，不指定朝向；等待当前原生规划器结果。')
    })().catch(error => { if (!cancelled) setNotice(String(error)) })
    return () => { cancelled = true }
  }, [])
  const state = {
    connected: true, events: [], session: { env: 'real', product: 'nav', mode: 'navigation', active_map: map, product_session_id: sessionId },
    odometry: { type: 'odometry', frame_id: 'map', x: request.start[0], y: request.start[1], z: request.start[2], yaw: 0, vx: 0, vy: 0, vz: 0, wz: 0, ts: now / 1000 },
    navigationStatus: { ...navigation, ts: now / 1000 },
    stateSnapshot: { ts: now / 1000, localization: { state: 'tracking', runtime_instance_id: 'recorded-903-pose', map_frame_jump_sequence: 0 } },
    stateSnapshotReceivedAt: now, globalPath: null, localPath: null, mapCloud: null, mapScene: null,
    sceneGraph: null, robotStatus: null, nativeTraversability: null, locations: null,
  } as unknown as SSEState
  return <main style={{ height: '100vh', display: 'flex', flexDirection: 'column' }}>
    <header style={{ background: '#152b36', color: 'white', padding: '12px 20px', fontSize: 14 }}>
      <b>903 保存地图 · 当前 UI · 离线规划验证</b>
      <span style={{ marginLeft: 20 }}>使用已记录起点；未连接机器人，不能下发行走。</span>
      {notice && <span style={{ marginLeft: 20 }}>{notice}</span>}
    </header>
    <nav id="scene-actions-slot" style={{ display: 'flex', alignItems: 'center', gap: 12, padding: '8px 20px' }} />
    <div style={{ flex: 1, minHeight: 0, position: 'relative', display: 'flex', flexDirection: 'column' }}>
      <SceneView workspace="navigation" sseState={state} locale="zh" showToast={setNotice}
        motionStartAllowed={false} motionStartBlockedReason="离线预览，不连接机器人" instructionActive={false}
        onOpenSavedMap={() => setNotice('当前展示的是本地 903 原始保存地图')}
        onStartMapping={() => {}} productSwitchAllowed={false} productSwitchReason="离线验证" />
    </div>
  </main>
}
createRoot(document.getElementById('root')!).render(<Preview />)
