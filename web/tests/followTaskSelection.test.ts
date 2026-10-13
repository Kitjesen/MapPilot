import assert from 'node:assert/strict'
import test from 'node:test'

import { selectDisplayedFollowTask } from '../src/services/followTask.ts'

test('a new Follow receipt stays visible until its matching SSE arrives', () => {
  const session = 'product-session-a'
  const terminalF1 = {
    follow_task_id: 'follow-1',
    product_session_id: session,
    task_terminal: true,
  }
  const acceptedF2 = {
    follow_task_id: 'follow-2',
    product_session_id: session,
  }

  assert.equal(selectDisplayedFollowTask(terminalF1, null, session), null)
  assert.deepEqual(
    selectDisplayedFollowTask(terminalF1, acceptedF2, session),
    acceptedF2,
  )
  assert.deepEqual(
    selectDisplayedFollowTask({ ...acceptedF2, task_terminal: false }, acceptedF2, session),
    acceptedF2,
  )
})

test('an older non-terminal status cannot mask a newer accepted Follow', () => {
  const session = 'product-session-a'
  const acceptedF2 = {
    follow_task_id: 'follow-2',
    product_session_id: session,
  }

  assert.deepEqual(
    selectDisplayedFollowTask({
      follow_task_id: 'follow-1',
      product_session_id: session,
      task_terminal: false,
    }, acceptedF2, session),
    acceptedF2,
  )
})

test('status from another Product session is never displayed', () => {
  assert.equal(
    selectDisplayedFollowTask({
      follow_task_id: 'follow-old-session',
      product_session_id: 'product-session-old',
      task_terminal: false,
    }, null, 'product-session-current'),
    null,
  )
})
