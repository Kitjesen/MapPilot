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
  assert.match(previews[0].getText(file), /aria-label="收起相机"/)
  assert.match(previews[0].getText(file), /setCameraPreference\(false\)/)
  assert.match(previews[0].getText(file), /getElementById\('scene-camera-toggle'\)\?\.focus\(\)/)
  assert.match(css, /\.canvasControls\s*\{[^}]*left:\s*16px/s)
  assert.match(css, /\.cameraPip\s*\{[^}]*position:\s*absolute;[^}]*top:\s*16px;[^}]*right:\s*16px/s)
  assert.match(css, /\.cameraPipImg\s*\{[^}]*object-fit:\s*contain/s)
  assert.match(css, /\.cameraPipVideoPending\s*\{[^}]*opacity:\s*0/s)
})

test('fullscreen keeps the global operator controls and restores the inspector on exit', async () => {
  const toggleBody = source.match(/const toggleSceneFullscreen = async \(\) => \{([\s\S]*?)\n {2}\}/)?.[1]
  const updateBody = source.match(/const updateFullscreen = \(\) => \{([\s\S]*?)\n {4}\}/)?.[1]
  assert.ok(toggleBody)
  assert.ok(updateBody)
  const preference = { current: null as boolean | null }
  let inspector = true
  let drawer = true
  let fullscreen = false
  const notices: string[] = []
  const document = {
    fullscreenElement: null as object | null,
    documentElement: { async requestFullscreen() { document.fullscreenElement = document.documentElement; update() } },
    async exitFullscreen() { document.fullscreenElement = null; update() },
  }
  const update = new Function('document', 'setSceneFullscreen', 'fullscreenInspector', 'setInspectorOpen', updateBody)
    .bind(null, document, (value: boolean) => { fullscreen = value }, preference, (value: boolean) => { inspector = value })
  const toggle = new Function('document', 'fullscreenInspector', 'inspectorOpen', 'setInspectorOpen', 'setDrawerOpen', 'showToast', `return async () => {${toggleBody}}`)(
    document, preference, inspector, (value: boolean) => { inspector = value },
    (value: boolean) => { drawer = value }, (notice: string) => notices.push(notice),
  )
  await toggle()
  assert.equal(document.fullscreenElement, document.documentElement, 'Stop and the camera must be within the fullscreen element')
  assert.equal(fullscreen, true)
  assert.equal(inspector, false)
  assert.equal(drawer, false)
  await document.exitFullscreen()
  assert.equal(fullscreen, false)
  assert.equal(inspector, true, 'browser Escape and the exit button use the same fullscreenchange restoration')
  assert.equal(preference.current, null)
  assert.deepEqual(notices, [])
  assert.match(source, /document\.addEventListener\('fullscreenchange', updateFullscreen\)/)
  assert.match(source, /document\.removeEventListener\('fullscreenchange', updateFullscreen\)/)
  assert.match(source, /goalSendPending \|\| document\.fullscreenElement\) return/)
  assert.match(css, /\.sceneView\.sceneFullscreen\s*\{[^}]*position:\s*fixed;[^}]*inset:\s*0;/s)
})

test('a rejected fullscreen request keeps the ordinary layout and reports the failure', async () => {
  const body = source.match(/const toggleSceneFullscreen = async \(\) => \{([\s\S]*?)\n {2}\}/)?.[1]
  assert.ok(body)
  let mutations = 0
  const preference = { current: null as boolean | null }
  const notices: string[] = []
  const document = { fullscreenElement: null, documentElement: { requestFullscreen: async () => { throw new Error('denied') } } }
  const toggle = new Function('document', 'fullscreenInspector', 'inspectorOpen', 'setInspectorOpen', 'setDrawerOpen', 'showToast', `return async () => {${body}}`)(
    document, preference, true, () => { mutations++ }, () => { mutations++ }, (notice: string) => notices.push(notice),
  )
  await toggle()
  assert.equal(mutations, 0)
  assert.equal(preference.current, null)
  assert.equal(notices.length, 1)
})
