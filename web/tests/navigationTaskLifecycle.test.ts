import assert from 'node:assert/strict'
import test from 'node:test'

import {
  describeNavigationTask,
  isTaskControlAwaitingConfirmation,
} from '../src/services/navigationTaskLifecycle.ts'

test('pause and resume ACKs remain pending until native lifecycle confirmation', () => {
  const pauseRequested = {
    taskId: 'task-1',
    action: 'pause' as const,
    state: 'requested' as const,
  }
  const resumeRequested = {
    taskId: 'task-1',
    action: 'resume' as const,
    state: 'requested' as const,
  }

  assert.equal(
    isTaskControlAwaitingConfirmation(
      pauseRequested,
      'task-1',
      describeNavigationTask({ state_name: 'PATH_ACTIVE' }),
    ),
    true,
  )
  assert.equal(
    isTaskControlAwaitingConfirmation(
      pauseRequested,
      'task-1',
      describeNavigationTask({ lifecycle_state_name: 'PAUSED' }),
    ),
    false,
  )
  assert.equal(
    isTaskControlAwaitingConfirmation(
      resumeRequested,
      'task-1',
      describeNavigationTask({ lifecycle_state_name: 'PAUSED' }),
    ),
    true,
  )
  assert.equal(
    isTaskControlAwaitingConfirmation(
      resumeRequested,
      'task-1',
      describeNavigationTask({ state_name: 'PATH_ACTIVE' }),
    ),
    false,
  )
  assert.equal(
    isTaskControlAwaitingConfirmation(
      resumeRequested,
      'task-1',
      describeNavigationTask({ lifecycle_state_name: 'RECOVERING' }),
    ),
    false,
  )
})

test('a confirmed paused task stays active and offers resume without hiding cancel', () => {
  const task = describeNavigationTask({ lifecycle_state_name: 'PAUSED', state: 3 })


  assert.equal(task.state, 'PAUSED')
  assert.equal(task.terminal, false)
  assert.equal(task.canPause, false)
  assert.equal(task.canResume, true)
  assert.equal(task.canCancel, true)
})

test('planning offers cancel only while executing and recovering offer pause', () => {
  const planning = describeNavigationTask({ lifecycle_state_name: 'PLANNING' })
  const executing = describeNavigationTask({ lifecycle_state_name: 'EXECUTING' })
  const recovering = describeNavigationTask({ lifecycle_state_name: 'RECOVERING' })

  assert.deepEqual(
    [planning.canPause, planning.canResume, planning.canCancel],
    [false, false, true],
  )
  assert.deepEqual(
    [executing.canPause, executing.canResume, executing.canCancel],
    [true, false, true],
  )
  assert.deepEqual(
    [recovering.canPause, recovering.canResume, recovering.canCancel],
    [true, false, true],
  )
})

test('a bare numeric state never invents a terminal lifecycle meaning', () => {
  const ambiguous = describeNavigationTask({ state: 3 })

  assert.equal(ambiguous.state, 'UNKNOWN')
  assert.equal(ambiguous.terminal, false)
  assert.equal(ambiguous.canPause, false)
  assert.equal(ambiguous.canResume, false)
  assert.equal(ambiguous.canCancel, true)

  const explicitlyTerminal = describeNavigationTask({ state: 3, terminal: true })
  assert.equal(explicitlyTerminal.terminal, true)
})
test('canonical lifecycle names win while legacy named states stay compatible', () => {
  const canonical = describeNavigationTask({
    lifecycle_state_name: 'PAUSED',
    state_name: 'REACHED',
  })
  const activeAlias = describeNavigationTask({ state_name: 'PATH_ACTIVE' })
  const successAlias = describeNavigationTask({ state_name: 'REACHED' })

  assert.equal(canonical.state, 'PAUSED')
  assert.equal(canonical.terminal, false)
  assert.equal(activeAlias.state, 'EXECUTING')
  assert.equal(successAlias.state, 'SUCCESS')
  assert.equal(successAlias.terminal, true)
})

test('idle is a canonical no-active-task state without task controls', () => {
  const idle = describeNavigationTask({ lifecycle_state_name: 'IDLE' })

  assert.equal(idle.state, 'IDLE')
  assert.equal(idle.terminal, false)
  assert.equal(idle.canPause, false)
  assert.equal(idle.canResume, false)
  assert.equal(idle.canCancel, false)
})
