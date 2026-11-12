import assert from 'node:assert/strict'
import test from 'node:test'
import { BOT_SHAPES, BOT_COLORS, BOT_PROFILE_KEY, DEFAULT_BOT_PROFILE, botPersona, readBotProfile, saveBotProfile, validateBotProfile, type BotProfile } from '../src/services/botProfile.ts'

test('appearance is local and persona carries only name and role', () => {
  const profile = validateBotProfile({ ...DEFAULT_BOT_PROFILE, name: ' 巡检伙伴 ', instructions: '简洁中文回答', color: 'blue', shape: 'square' })
  assert.equal(profile.name, '巡检伙伴')
  assert.match(botPersona(profile), /巡检伙伴.*\n.*简洁中文回答/)
  assert.doesNotMatch(botPersona(profile), /blue|square|full_access/)
  assert.equal(botPersona(DEFAULT_BOT_PROFILE), '')
  assert(botPersona({ ...profile, instructions: '字'.repeat(1600) }).length <= 2000)
})
test('profile validation rejects unknown forms, blank names, and oversized instructions', () => {
  for (const invalid of [{ name: '' }, { shape: 'unknown' }, { color: 'red' }, { size: 57 }, { instructions: 'x'.repeat(1601) }, { name: 'x'.repeat(33) }]) {
    assert.throws(() => validateBotProfile({ ...DEFAULT_BOT_PROFILE, ...invalid }))
  }
})

test('older appearance settings gain a default size and keep their selected tone', () => {
  const profile = validateBotProfile({ name: 'Scout', shape: 'capsule', color: 'blue', instructions: '' })
  assert.equal(profile.size, 48)
  assert.equal(profile.color, 'blue')
  assert.equal(profile.shape, 'capsule')
})
test('profile reloads; a storage failure is not reported as saved', () => {
  const storage = new Map<string, string>()
  Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: { getItem: (key: string) => storage.get(key), setItem: (key: string, value: string) => storage.set(key, value) } })
  try {
    saveBotProfile({ ...DEFAULT_BOT_PROFILE, name: 'Scout', shape: 'square', color: 'sage', size: 64 })
    assert.equal(readBotProfile().name, 'Scout')
    assert.equal(readBotProfile().shape, 'square')
    assert.equal(readBotProfile().color, 'sage')
    assert.equal(readBotProfile().size, 64)
    for (const shape of Object.keys(BOT_SHAPES) as BotProfile['shape'][]) {
      for (const color of Object.keys(BOT_COLORS) as BotProfile['color'][]) {
        saveBotProfile({ ...DEFAULT_BOT_PROFILE, shape, color })
        assert.equal(readBotProfile().shape, shape)
        assert.equal(readBotProfile().color, color)
      }
    }
    storage.set(BOT_PROFILE_KEY, '{broken')
    assert.deepEqual(readBotProfile(), DEFAULT_BOT_PROFILE)
    Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: { setItem() { throw new Error('quota') } } })
    assert.throws(() => saveBotProfile(DEFAULT_BOT_PROFILE), /quota/)
  } finally { Reflect.deleteProperty(globalThis, 'localStorage') }
})
