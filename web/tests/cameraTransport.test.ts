import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const feedSource = readFileSync(
  new URL('../src/components/CameraFeed.tsx', import.meta.url),
  'utf8',
)
const cameraHookSource = readFileSync(
  new URL('../src/hooks/useCamera.ts', import.meta.url),
  'utf8',
)
const feedStyles = readFileSync(
  new URL('../src/components/CameraFeed.module.css', import.meta.url),
  'utf8',
)
const sceneStyles = readFileSync(
  new URL('../src/components/SceneView.module.css', import.meta.url),
  'utf8',
)

test('React camera keeps the Gateway JPEG WebSocket as its reliable path', () => {
  assert.match(feedSource, /useWHEP/)
  assert.match(feedSource, /useCamera/)
  assert.match(feedSource, /'\/ws\/camera'/)
  assert.doesNotMatch(feedSource, /useWebRTC|\/api\/v1\/webrtc\/offer/)
  assert.match(feedSource, /useCamera\(whepFailed \? '\/ws\/camera' : ''\)/)
  assert.match(cameraHookSource, /new WebSocket\(wsUrl\)/)
  assert.match(cameraHookSource, /setFrameCount\(n => n \+ 1\)/)
})

test('camera views preserve the upright orientation supplied by Gateway', () => {
  assert.doesNotMatch(feedStyles, /\.img\s*\{[^}]*rotate\(/s)
  assert.doesNotMatch(sceneStyles, /\.cameraPipImg\s*\{[^}]*rotate\(/s)
  assert.match(sceneStyles, /\.cameraPipImg\s*\{[^}]*aspect-ratio:\s*4\s*\/\s*3/s)
  assert.match(feedSource, /className=\{`\$\{styles\.img\} \$\{styles\.imgWhep\}`\}/)
  assert.match(feedStyles, /\.imgWhep\s*\{[^}]*rotate\(180deg\)/s)
})
