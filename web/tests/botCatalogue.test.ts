import assert from 'node:assert/strict'
import test from 'node:test'
import { BOT_SHAPES, BOT_SHAPE_GROUPS, GROK_SHAPES, LINGTU_HEADS } from '../src/services/botShapes.ts'
import { bodyFor, renderAvatarSvg } from '../src/vendor/grok-avatar/src/index.mjs'

test('the complete catalogue preserves classic profiles and registers every shape exactly once', () => {
  assert.equal(Object.keys(BOT_SHAPES).length, 52)
  assert.deepEqual(BOT_SHAPE_GROUPS.map(group => group.shapes.length), [26, 18, 8])
  const entries = BOT_SHAPE_GROUPS.flatMap(group => group.shapes)
  assert.equal(new Set(entries).size, 52)
  assert.deepEqual(new Set(entries), new Set(Object.keys(BOT_SHAPES)))
  for (const shape of entries) assert.equal(Number(Object.hasOwn(GROK_SHAPES, shape)) + Number(Object.hasOwn(LINGTU_HEADS, shape)), 1)
  assert.deepEqual(BOT_SHAPE_GROUPS.at(-1)?.shapes, ['orb', 'capsule', 'square', 'bean', 'drop', 'cloud', 'bud', 'pebble'])
})

test('all 26 reference shapes have distinct bodies and produce finite SVG geometry at supported sizes', () => {
  assert.equal(new Set(Object.values(GROK_SHAPES).map(item => bodyFor(item.shape))).size, 26)
  for (const { shape } of Object.values(GROK_SHAPES)) {
    for (const size of [32, 48, 64]) {
      const svg = renderAvatarSvg({ shape, color: '#88add4', size, isStatic: true })
      assert.match(svg, /<svg\b/)
      assert.match(svg, /<path\b/)
      assert.doesNotMatch(svg, /NaN|Infinity|undefined/)
    }
  }
})
