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
const videoFrameHookSource = readFileSync(
  new URL('../src/hooks/useVideoFrame.ts', import.meta.url),
  'utf8',
)
const rotatedVideoHookSource = readFileSync(
  new URL('../src/hooks/useRotatedVideoLayout.ts', import.meta.url),
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
  assert.doesNotMatch(feedSource, /useWebRTC\(|\/api\/v1\/webrtc\/offer/)
  assert.match(feedSource, /useCamera\(useWebRTC \? '' : '\/ws\/camera'\)/)
  assert.doesNotMatch(feedSource, /whepFailed|8000/)
  assert.match(cameraHookSource, /new WebSocket\(wsUrl\)/)
  assert.match(cameraHookSource, /setFrameCount\(n => n \+ 1\)/)
  assert.match(feedSource, /whepConnected && videoFrame\.hasFrame && videoLayout\.ready/)
  assert.match(videoFrameHookSource, /requestVideoFrameCallback/)
  assert.match(videoFrameHookSource, /video\.srcObject = null/)
})

test('camera views preserve the upright orientation supplied by Gateway', () => {
  assert.doesNotMatch(feedStyles, /\.img\s*\{[^}]*rotate\(/s)
  assert.doesNotMatch(sceneStyles, /\.cameraPipImg\s*\{[^}]*rotate\(/s)
  assert.match(sceneStyles, /\.cameraPipMedia\s*\{[^}]*aspect-ratio:\s*4\s*\/\s*3/s)
  assert.match(feedSource, /className=\{`\$\{styles\.img\} \$\{styles\.imgWhep\}/)
  assert.doesNotMatch(feedStyles, /\.imgWhep\s*\{[^}]*rotate\(/s)
  assert.match(feedSource, /useRotatedVideoLayout/)
  assert.match(rotatedVideoHookSource, /fitRotatedVideo/)
})
