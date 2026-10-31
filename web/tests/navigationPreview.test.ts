import assert from 'node:assert/strict'
import test from 'node:test'
import { previewNavigationPlan } from '../src/services/api.ts'
import { navigationPreviewErrorMessage, navigationPreviewIsCurrent, sceneGoalHeight, navigationSupportHeightFromRunPlan } from '../src/services/navigationPreview.ts'
import type { PlanPreviewResponse } from '../src/types/index.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

test('preview service errors stay distinct from infeasible paths and editable target validation', () => {
  assert.equal(navigationPreviewErrorMessage('规划失败：local_review_preview_service_unavailable'), '路线预览服务不可用')
  assert.equal(navigationPreviewErrorMessage('规划失败：Failed to fetch'), '路线预览服务不可用')
  assert.equal(navigationPreviewErrorMessage('无法生成路线：collision_at_trajectory_start'), '无法生成路线：轨迹起点存在碰撞，运动指令未获放行。')
  assert.equal(navigationPreviewErrorMessage('无法生成路线：support_disconnected'), '无法生成路线：起点与目标之间的地面支撑尚未连通。')
  assert.equal(navigationPreviewErrorMessage('规划失败：collision_stale'), '规划失败：等待实时雷达地图更新。')
  assert.equal(navigationPreviewErrorMessage('无法生成路线：等待实时雷达地图更新。'), '无法生成路线：等待实时雷达地图更新。')
  assert.equal(navigationPreviewErrorMessage('未找到可行路线'), '未找到可行路线')
  assert.equal(navigationPreviewErrorMessage('请输入目标高度'), '请输入目标高度')
  assert.equal(navigationPreviewErrorMessage('连接断开，等待恢复'), '连接断开，等待恢复')
  assert.equal(navigationPreviewErrorMessage('发送目标失败: goal_terminal_pending'), '目标提交失败')
})

test('the direct plan preview API forwards the effective acceptance radius', async () => {
  let body: Record<string, unknown> | null = null
  globalThis.fetch = async (_input, init) => {
    body = JSON.parse(String(init?.body)) as Record<string, unknown>
    return Response.json({
      schema_version: 1,
      ok: true,
      feasible: false,
      frame_id: 'map',
      start: null,
      goal: { x: 1, y: 2, z: 0.3 },
      path: [],
      count: 0,
      source: 'native_nav',
      reasons: ['no_path'],
      ts: 1,
    })
  }

  await previewNavigationPlan(1, 2, 0.3, 0.25)

  assert.deepEqual(body, {
    x: 1,
    y: 2,
    z: 0.3,
    acceptance_radius_m: 0.25,
  })
})

test('a preview cannot authorize a path from a robot pose that has since moved', () => {
  const preview = { feasible: true, frame_id: 'map', start: { x: 1, y: 2, z: .2 } } as PlanPreviewResponse
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1.1, y: 2, z: .2 }), true)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 2, y: 2, z: .2 }), false)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1, y: 2, z: 1.2 }), false)
  assert.equal(navigationPreviewIsCurrent(preview, null), false)
  assert.equal(navigationPreviewIsCurrent(preview, { x: 1, y: 2, z: null }), false)
  assert.equal(navigationPreviewIsCurrent({ ...preview, frame_id: 'odom' }, preview.start), false)
  assert.equal(navigationPreviewIsCurrent({ ...preview, feasible: false }, preview.start), false)
})

test('planar map clicks preserve continuous body height across the old 20 cm slice boundary', () => {
  assert.equal(sceneGoalHeight(-.01), -.01)
  assert.equal(sceneGoalHeight(.01), .01)
  assert.equal(sceneGoalHeight(.0184659), .0184659)
})

test('an explicit 3D target keeps its elevation while missing height cannot become z=0', () => {
  assert.equal(sceneGoalHeight(.02, 1.25), 1.25)
  assert.equal(sceneGoalHeight(.02, 0), 0)
  for (const z of [null, undefined, NaN, Infinity]) assert.equal(sceneGoalHeight(z), null)
  assert.equal(sceneGoalHeight(.02, NaN), null)
})

test('surface picks use the selected RunPlan support height once across levels', () => {
  for (const support of [.435, .32, 0]) {
    const height = navigationSupportHeightFromRunPlan({ host: { config: { octoplanner3d_support_height_m: support } } })
    for (const floor of [0, 1.8, 3.6]) {
      const target = sceneGoalHeight(.435, floor, 'surface', height)
      assert.equal(target, floor + support)
      assert.equal(sceneGoalHeight(.435, target!), target, 'manual/preview body coordinates must not be raised again')
    }
  }
  for (const value of [undefined, null, NaN, Infinity, -.1, '.435']) {
    const height = navigationSupportHeightFromRunPlan({ host: { config: { octoplanner3d_support_height_m: value } } })
    assert.equal(height, null)
    assert.equal(sceneGoalHeight(.435, 3.6, 'surface', height), null)
  }
  assert.equal(sceneGoalHeight(.435, undefined, 'surface', .435), null)
  assert.equal(sceneGoalHeight(.435, 3.6, 'plane', null), 3.6)
})
