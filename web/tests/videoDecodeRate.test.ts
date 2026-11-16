import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

test('video decode rate is displayed only for WebRTC playback', async () => {
  const feed = await import('node:fs/promises').then(fs => fs.readFile('src/components/CameraFeed.tsx', 'utf8'))
  const scene = await import('node:fs/promises').then(fs => fs.readFile('src/components/SceneView.tsx', 'utf8'))
  assert.match(feed, /useVideoDecodeRate\(videoRef, whepStream, whepConnected\)/)
  assert.match(feed, /videoFps=\{videoFps\}/)
  assert.match(scene, /cameraDecodeFps/)
})

test('decode sampling clears at stream cleanup and disabled playback hides the rate', () => {
  const source = ts.transpileModule(readFileSync(new URL('../src/hooks/useVideoDecodeRate.ts', import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  const exports: Record<string, unknown> = {}
  let fps: number | null = null
  let effect: () => (() => void) | undefined = () => undefined
  let observe: (now: number) => void = () => {}
  let cancelled = 0
  runInNewContext(source, { exports, performance: { now: () => 0 }, require: () => ({
    useState: () => [fps, (next: number | null) => { fps = next }],
    useEffect: (setup: typeof effect) => { effect = setup },
  }) })
  const video = { srcObject: null, requestVideoFrameCallback: (callback: typeof observe) => { observe = callback; return 1 },
    cancelVideoFrameCallback: () => { cancelled++ } }
  const stream = {}
  const rate = exports.useVideoDecodeRate as (ref: object, stream: object, enabled: boolean) => number | null
  assert.equal(rate({ current: video }, stream, true), null)
  const cleanup = effect()!
  observe(1000)
  assert.equal(rate({ current: video }, stream, true), 1)
  assert.equal(rate({ current: video }, stream, false), null)
  cleanup()
  assert.equal(fps, null)
  assert.equal(cancelled, 1)
  assert.equal(video.srcObject, null)
  assert.equal(rate({ current: video }, stream, true), null)
})
