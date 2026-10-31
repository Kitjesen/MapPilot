import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import { VideoFrameClock, videoFrameFreshness } from '../src/hooks/useVideoFrame.ts'

test('a stream that never decodes its first frame becomes stalled after the attachment deadline', () => {
  assert.deepEqual(videoFrameFreshness(1000, null, 3999, 3000), {
    hasFrame: false, lastFrameAt: null, stalled: false,
  })
  assert.deepEqual(videoFrameFreshness(1000, null, 4001, 3000), {
    hasFrame: false, lastFrameAt: null, stalled: true,
  })
})

test('delayed ICE does not consume the connected first-frame window in either camera view', () => {
  const trackReceivedAt = 1000
  const connectedAt = trackReceivedAt + 6000
  const clock = new VideoFrameClock(connectedAt)

  assert.equal(clock.tick(connectedAt, 3000), null)
  assert.equal(clock.tick(connectedAt + 2999, 3000), null)
  assert.deepEqual(clock.observe(connectedAt + 3000), {
    hasFrame: true, lastFrameAt: connectedAt + 3000, stalled: false,
  })
  assert.deepEqual(clock.tick(connectedAt + 6001, 3000), {
    hasFrame: false, lastFrameAt: null, stalled: true,
  })

  const feed = readFileSync(new URL('../src/components/CameraFeed.tsx', import.meta.url), 'utf8')
  const scene = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
  assert.match(feed, /useVideoFrame\(videoRef, whepStream, whepConnected\)/)
  assert.match(scene, /useVideoFrame\(\s*cameraVideoRef,\s*whepStream,\s*cameraVisible && sseState\.connected && whepConnected,/)
})

test('a decoded stream becomes stalled after its last frame stops advancing', () => {
  assert.deepEqual(videoFrameFreshness(1000, 2500, 5499, 3000), {
    hasFrame: true, lastFrameAt: 2500, stalled: false,
  })
  assert.deepEqual(videoFrameFreshness(1000, 2500, 5501, 3000), {
    hasFrame: false, lastFrameAt: null, stalled: true,
  })
})

test('decoded frames stay local while UI state publishes at health-check frequency', () => {
  const clock = new VideoFrameClock(1000)
  assert.deepEqual(clock.observe(1100), {
    hasFrame: true, lastFrameAt: 1100, stalled: false,
  })
  for (let timestamp = 1133; timestamp < 2000; timestamp += 33) {
    assert.equal(clock.observe(timestamp), null)
  }
  assert.deepEqual(clock.tick(2000, 3000), {
    hasFrame: true, lastFrameAt: 1991, stalled: false,
  })
  assert.equal(clock.tick(2500, 3000), null)
  assert.deepEqual(clock.tick(5000, 3000), {
    hasFrame: false, lastFrameAt: null, stalled: true,
  })
})
