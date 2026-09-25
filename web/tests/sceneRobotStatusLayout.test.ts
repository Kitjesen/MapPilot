import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const sceneSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const sceneStyles = readFileSync(
  new URL('../src/components/SceneView.module.css', import.meta.url),
  'utf8',
)

test('robot status normalizes negative zero before rendering telemetry', () => {
  assert.match(sceneSource, /normalizeDisplayZero/)
  assert.doesNotMatch(sceneSource, /displayRobot[XY]\s*=.*?\.toFixed\(2\)/)
})

test('robot status value slots reserve stable widths', () => {
  assert.match(sceneSource, /styles\.robotPositionValue/)
  assert.match(sceneSource, /styles\.robotYawValue/)
  assert.match(sceneSource, /styles\.robotSpeedValue/)
  assert.match(sceneStyles, /\.robotPositionValue[\s\S]*?inline-size:/)
  assert.match(sceneStyles, /\.robotYawValue[\s\S]*?inline-size:/)
  assert.match(sceneStyles, /\.robotSpeedValue[\s\S]*?inline-size:/)
})

test('navigation details render all operator axes from navigation status', () => {
  assert.match(sceneSource, /navigationStatus = liveNavigationStatus\(sseState, localNowS\)/)
  assert.match(sceneSource, /presentNavigationStatus\(navigationStatus, locale\)/)
  assert.match(sceneSource, /navigationView\.task\.label/)
  assert.match(sceneSource, /navigationView\.goalAdmission\.label/)
  assert.match(sceneSource, /navigationView\.control\.label/)
  assert.match(sceneSource, /navigationView\.motion\.permission\.label/)
  assert.match(sceneSource, /navigationView\.motion\.stopConfirmation\.label/)
  assert.doesNotMatch(sceneSource, /navigationView\.summary/)
  assert.doesNotMatch(sceneSource, /missionStatus/)
  assert.doesNotMatch(sceneSource, /navigationStatus\?\.target/)
})

test('scene inspector provides one selected keyboard tab and hides inactive panels', () => {
  assert.match(sceneSource, /role="tablist" aria-label="现场面板内容"/)
  assert.match(sceneSource, /\['ArrowLeft', 'ArrowRight', 'Home', 'End'\]/)
  assert.match(sceneSource, /tabIndex=\{inspectorTab === item.key \? 0 : -1\}/)
  assert.match(sceneSource, /tabs\[next\]\?\.focus\(\)/)
  for (const tab of ['status', 'layers', 'tools']) {
    assert.ok(sceneSource.includes(`hidden={inspectorTab !== '${tab}'}`))
  }
  assert.match(sceneStyles, /\.sceneView \[hidden\] \{ display: none !important; \}/)
})

test('the canvas click that dismisses a menu cannot select a goal, while the next click can', () => {
  const downBody = sceneSource.match(/onPointerDownCapture=\{\(\) => \{([\s\S]*?)\}\}/)?.[1]
  const upBody = sceneSource.match(/onMouseUpCapture=\{event => \{([\s\S]*?)\}\}/)?.[1]
  assert.ok(downBody)
  assert.ok(upBody)
  const down = new Function('document', 'dismissOnlyCanvasClick', downBody.replace('<HTMLDetailsElement>', ''))
  const up = new Function('event', 'dismissOnlyCanvasClick', upBody)
  const ref = { current: false }
  const menu = { open: true }
  const document = { querySelectorAll: (selector: string) => {
    assert.equal(selector, 'details[name="lingtu-menu"][open]')
    return menu.open ? [menu] : []
  } }
  let stopped = 0
  const event = { shiftKey: true, preventDefault() {}, stopPropagation() { stopped++ } }
  down(document, ref)
  assert.equal(menu.open, false, 'canvas pointerdown must close the open menu')
  up(event, ref)
  assert.equal(stopped, 1, 'document menu dismissal must not erase the captured click guard')
  down(document, ref)
  up(event, ref)
  assert.equal(stopped, 1, 'the next ordinary or Shift click must propagate normally')
  menu.open = true
  down(document, ref)
  assert.equal(menu.open, false)
  up(event, ref)
  assert.equal(stopped, 2, 'touch-generated compatibility mouseup follows the same pointerdown guard')
})

test('scene retains operational controls while showing failures outside collapsed details', () => {
  assert.equal((sceneSource.match(/onClick=\{handleSaveMap\}/g) ?? []).length, 1)
  assert.match(sceneSource, /onClick=\{handleRestartLocalization\}/)
  assert.match(sceneSource, /onClick=\{handleGlobalRelocalize\}/)
  assert.match(sceneSource, /onClick=\{handleRelocalize\}/)
  assert.match(sceneSource, /!observe && hasGoal && <div className=\{styles.motionActions\}/)
  assert.doesNotMatch(sceneSource, /onClick=\{handleStop\}/)
  const alertStart = sceneSource.indexOf('styles.sceneAlert}')
  assert.ok(alertStart > sceneSource.indexOf('styles.canvasHeader'))
  assert.ok(alertStart < sceneSource.indexOf('<Scene3D', alertStart))
  assert.doesNotMatch(sceneSource, /styles\.sceneHeading/)
  assert.match(sceneSource, /!operatorAttention \? styles.sceneAlertQuiet/)
  assert.match(sceneSource, /plannerAttention \|\| \(sseState.connected/)
  assert.match(sceneSource, /<summary>位姿详情<\/summary>/)
  assert.match(sceneSource, /<summary>控制详情<\/summary>/)
  assert.match(sceneSource, /<summary>雷达详情<\/summary>/)
})

test('mapping opens on the cumulative map and keeps the local projection explicitly non-traversable', () => {
  assert.match(sceneSource, /useState<'coverage' \| 'points' \| 'global'>\('global'\)/)
  assert.match(sceneSource, /mappingView === 'coverage'\) scene3DRef\.current\?\.topView\(6\)/)
  assert.match(sceneSource, />局部投影<\/button>/)
  assert.match(sceneSource, />局部地图<\/button>/)
  assert.match(sceneSource, />整图<\/button>/)
  assert.match(sceneSource, /<details className=\{styles.mappingReadingKey\} aria-label="建图显示图例">/)
  assert.match(sceneSource, /<summary>图例<\/summary>/)
  assert.match(sceneSource, /地面相对高度 · 不代表可通行/)
  assert.match(sceneSource, /二维投影不是地面高度/)
  assert.match(sceneSource, /当前局部窗口 · 切换整图查看累计范围/)
  assert.match(sceneSource, /本次累计建图 · 显示经过采样/)
  assert.doesNotMatch(sceneSource, /完整 SLAM 建图只在保存地图后查看/)
  assert.doesNotMatch(sceneSource, /空间点云 · 累计预览/)
  assert.doesNotMatch(sceneSource, /topView\(10\)/)
})


test('keyboard mode stays beside the scene and outside the inspector', () => {
  const mode = sceneSource.indexOf('<TeleopPanel');
  assert.ok(mode < sceneSource.indexOf('<aside className={styles.sidePanel}'));
  assert.match(sceneSource, /teleopMode && !drawerOpen/);
  assert.match(sceneSource, /aria-pressed=\{teleopMode\}/);
  assert.doesNotMatch(sceneSource, /teleopPanelOpen|<FloatingWidget/);
});
