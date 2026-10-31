import assert from 'node:assert/strict'
import test from 'node:test'
import { createElement } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import { createServer } from 'vite'

test('SLAM panel renders the native geometry states without claiming unapplied mitigation', async () => {
  const server = await createServer({ server: { middlewareMode: true }, appType: 'custom' })
  try {
    const { SlamStatusPanel } = await server.ssrLoadModule('/src/components/SlamStatusPanel.tsx')
    const render = (geometry: Record<string, unknown>) => renderToStaticMarkup(createElement(
      SlamStatusPanel, {
        locale: 'zh', showToast: () => {},
        sseState: { session: null, slamDiag: { data: { ltlio_degeneracy: geometry } } },
      },
    ))
    const unavailable = render({ available: false, guidance: 'awaiting_lidar_geometry' })
    assert.match(unavailable, /退化诊断不可用/)
    assert.match(unavailable, /等待 LiDAR 几何/)
    assert.doesNotMatch(unavailable, /无弱方向/)
    const categories = ['UNOBSERVABLE', ...Array<string>(5).fill('OBSERVABLE')]
    const principal_directions = Array.from({ length: 6 }, (_, i) =>
      Array.from({ length: 6 }, (_, j) => i === 0 ? (j === 3 ? 1 : 0) : (j === i ? 1 : 0)))
    const weak = render({ available: true, categories, principal_directions,
      direction_frame: 'imu', rotation_scale_m: 1.2, update_applied: false,
      guidance: 'turn_or_retrace_for_features' })
    assert.match(weak, /UNOBSERVABLE/)
    assert.match(weak, /Rx Ry Rz \/ Tx Ty Tz/)
    assert.match(weak, /1\.00/)
    assert.match(weak, /转向或回退以获取特征/)
    assert.match(weak, /仅诊断，未抑制/)
    assert.doesNotMatch(weak, /已应用更新/)
    const applied = render({ available: true, categories, principal_directions,
      update_applied: true, guidance: 'turn_or_retrace_for_features' })
    assert.match(applied, /已应用更新/)
    assert.doesNotMatch(applied, /仅诊断，未抑制/)
    const healthy = render({ available: true, categories: Array<string>(6).fill('OBSERVABLE'),
      principal_directions, update_applied: false, guidance: 'none' })
    assert.match(healthy, /无弱方向/)
    assert.doesNotMatch(healthy, /UNOBSERVABLE/)
  } finally {
    await server.close()
  }
})
