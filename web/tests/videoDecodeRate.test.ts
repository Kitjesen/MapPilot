import test from 'node:test'
import assert from 'node:assert/strict'

test('video decode rate is displayed only for WebRTC playback', async () => {
  const feed = await import('node:fs/promises').then(fs => fs.readFile('src/components/CameraFeed.tsx', 'utf8'))
  const scene = await import('node:fs/promises').then(fs => fs.readFile('src/components/SceneView.tsx', 'utf8'))
  assert.match(feed, /useVideoDecodeRate\(videoRef, whepStream, whepConnected\)/)
  assert.match(feed, /videoFps=\{videoFps\}/)
  assert.match(scene, /cameraDecodeFps/)
})
