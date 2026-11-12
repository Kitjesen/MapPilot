import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import ts from 'typescript'

const source = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
const css = readFileSync(new URL('../src/components/SceneView.module.css', import.meta.url), 'utf8')
const file = ts.createSourceFile('SceneView.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)

test('one optional camera preview stays inside the map in normal and full-screen layouts', () => {
  const previews: ts.JsxElement[] = []
  const visit = (node: ts.Node) => {
    if (ts.isJsxElement(node) && node.openingElement.attributes.properties.some(attribute =>
      ts.isJsxAttribute(attribute) && attribute.name.getText(file) === 'aria-label'
      && attribute.initializer?.getText(file) === '"相机实时画面"')) previews.push(node)
    ts.forEachChild(node, visit)
  }
  visit(file)
  assert.equal(previews.length, 1, 'normal and fullscreen must share one preview, not duplicate video')
  const ancestorClasses: string[] = []
  for (let parent = previews[0].parent; parent; parent = parent.parent) {
    if (ts.isJsxElement(parent)) {
      const attribute = parent.openingElement.attributes.properties.find(attribute =>
        ts.isJsxAttribute(attribute) && attribute.name.getText(file) === 'className')
      if (attribute) ancestorClasses.push(attribute.getText(file))
      assert.notEqual(parent.openingElement.tagName.getText(file), 'aside')
    }
  }
  assert.ok(ancestorClasses.includes('className={styles.canvasWrap}'))
  assert.ok(!ancestorClasses.includes('className={styles.canvasControls}'), 'camera position must be independent of alert height')
  assert.match(source, /useState\(true\)\s*\n\s*const cameraVisible = cameraPreference/)
  assert.equal((source.match(/useCamera\(/g) ?? []).length, 1)
  assert.equal((source.match(/useWHEP\(/g) ?? []).length, 1)
  assert.equal((source.match(/useVideoFrame\(/g) ?? []).length, 1)
  assert.match(source, /whepConnected && cameraVideoFrame\.hasFrame && cameraVideoLayout\.ready/)
  assert.match(source, /cameraVisible && !useWebRTCCamera \? '\/ws\/camera' : ''/)
  assert.match(source, /whepStream \|\| \(jpegCameraRecovered && cameraImgSrc\)/)
  assert.match(source, /useRotatedVideoLayout/)
  assert.match(previews[0].getText(file), /aria-hidden=\{!cameraHasFrame\}/)
  assert.match(previews[0].getText(file), /cameraHasFrame && <div className=\{styles\.cameraPipHeader\}/)
  assert.doesNotMatch(previews[0].getText(file), /setCameraPreference/)
  assert.match(source, /cameraToggleRef\.current\?\.focus\(\)/)
  assert.match(css, /\.canvasControls\s*\{[^}]*left:\s*16px/s)
  assert.match(css, /\.cameraPip\s*\{[^}]*position:\s*absolute;[^}]*top:\s*calc\(var\(--camera-empty-top\) \+ 64px \+ var\(--robot-summary-height, 0px\)\);[^}]*right:\s*var\(--scene-tools-inset\)/s)
  assert.match(css, /\.cameraPipImg\s*\{[^}]*object-fit:\s*contain/s)
  assert.match(css, /\.cameraPipVideoPending\s*\{[^}]*opacity:\s*0/s)
  assert.match(css, /\.cameraDockEmpty\s*\{[^}]*opacity:\s*0;[^}]*pointer-events:\s*none/s)
  assert.doesNotMatch(css, /\.cameraDockEmpty\s*\{[^}]*(?:display:\s*none|visibility:\s*hidden)/s)
})

test('the fixed scene tools share one camera toggle outside the clipped media', () => {
  const group = source.match(/<div className=\{styles\.sceneToolGroup\}([\s\S]*?)<Scene3D/)?.[1]
  assert.ok(group)
  assert.match(group, /ref=\{cameraToggleRef\}[^>]*id="scene-camera-toggle"/)
  assert.equal((source.match(/id="scene-camera-toggle"/g) ?? []).length, 1)
  assert.match(group, /<span>相机<\/span>/)
  assert.match(group, /<span>图层<\/span>/)
  assert.doesNotMatch(group, /显示设置|viewMenu|全屏地图|控制与定位诊断/)
  assert.match(group, /cameraHasFrame \? '实时' : cameraStatus\?\.status === 'not_loaded' \? '未启用' : cameraLastFrameAt \? '中断' : '等待'/)
  assert.doesNotMatch(group, /styles\.teleopToggle|setTeleopMode/)
  assert.match(css, /\.mapTools\s*\{[^}]*top:\s*var\(--camera-empty-top\)/s)
  assert.match(css, /\.sceneToolGroup\s*\{[^}]*display:\s*flex/s)
  assert.doesNotMatch(css, /\.sceneToolGroup\s*\{[^}]*(?:overflow:\s*hidden|backdrop-filter|border:)/s)
  assert.match(css, /\.cameraToggle\s*\{[^}]*min-height:\s*44px/s)
})

