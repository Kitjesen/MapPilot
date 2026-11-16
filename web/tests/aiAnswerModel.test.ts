import assert from 'node:assert/strict'
import test from 'node:test'
import { answerBlocks, answerCopyText } from '../src/components/aiAnswerModel.ts'

test('AI answer copy removes markup and appends numbered sources', () => {
  assert.equal(
    answerCopyText('**Alfama** is best [1].\n\n- Dates [2].', [
      { name: 'Notion', title: 'Offsite budget' },
      { name: 'Slack', title: 'Team dates' },
    ]),
    'Alfama is best .\n\n- Dates .\n\nSources:\n1. Notion: Offsite budget\n2. Slack: Team dates',
  )
})

test('AI answer blocks preserve paragraph and list boundaries', () => {
  assert.deepEqual(answerBlocks('first\nline\n\n- one\n- two'), ['first\nline', '- one\n- two'])
})
