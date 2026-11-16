import assert from 'node:assert/strict'
import test from 'node:test'
import { BOT_ACTIVITIES, BOT_ACTIVITY_GROUPS, botActivityForTool } from '../src/components/agentConversation/botPresentation.ts'

test('Grok activity catalog keeps every renderer state grouped and mapped to real tools', () => {
  const grouped = BOT_ACTIVITY_GROUPS.flatMap(group => group.keys)
  assert.equal(Object.keys(BOT_ACTIVITIES).length, 52)
  assert.equal(new Set(grouped).size, 52)
  assert.equal(botActivityForTool('web_search'), 'searching')
  assert.equal(botActivityForTool('web_extract'), 'reading')
  assert.equal(botActivityForTool('run_python'), 'coding')
  assert.equal(botActivityForTool('navigate_to'), 'controlling')
  assert.equal(botActivityForTool('unknown_tool'), 'working')
  assert.equal(botActivityForTool(undefined), undefined)
})
