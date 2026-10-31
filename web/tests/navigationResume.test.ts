import assert from 'node:assert/strict'
import test from 'node:test'
import { resumeNavigation } from '../src/services/api.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

test('resume waits for control state to clear and never submits a goal', async () => {
  const requests: string[] = []
  let statusReads = 0
  globalThis.fetch = async (input, options) => {
    requests.push(`${options?.method ?? 'GET'} ${String(input)}`)
    return Response.json(options?.method === 'POST'
      ? { ok: true, status: 'motion_resume_acknowledged' }
      : {
          control: { authority: 'NONE', resume_required: ++statusReads < 2 },
          motion: { permission: 'CLEAR', reason: 'motion_clear' },
        })
  }
  assert.equal((await resumeNavigation()).ok, true)
  assert.deepEqual(requests, [
    'POST /api/v1/navigation/resume',
    'GET /api/v1/navigation/status',
    'GET /api/v1/navigation/status',
  ])
})

test('an accepted resume that leaves control held is a visible failure', async () => {
  globalThis.fetch = async (_input, options) => Response.json(options?.method === 'POST'
    ? { ok: true, status: 'motion_resume_acknowledged', native_reason: 'autonomy_already_ready' }
    : {
        control: { authority: 'NONE', resume_required: true },
        motion: { permission: 'HELD', reason: 'resume_required' },
      })
  await assert.rejects(resumeNavigation(), /控制状态仍未确认/)
})

test('resume does not claim success while authoritative control state is unknown', async () => {
  globalThis.fetch = async (_input, options) => Response.json(options?.method === 'POST'
    ? { ok: true, status: 'motion_resume_acknowledged' }
    : {
        control: { authority: 'UNKNOWN', resume_required: false },
        motion: { permission: 'UNKNOWN', reason: 'control_state_unknown' },
      })
  await assert.rejects(resumeNavigation(), /控制状态仍未确认/)
})

for (const blocker of ['stale_collision_map', 'navigation_session_inactive']) {
  test(`control resume succeeds independently of ${blocker}`, async () => {
    const requests: string[] = []
    globalThis.fetch = async (input, options) => {
      requests.push(`${options?.method ?? 'GET'} ${String(input)}`)
      return Response.json(options?.method === 'POST'
        ? { ok: true, status: 'motion_resume_acknowledged' }
        : {
            control: { authority: 'NONE', resume_required: false },
            goal_admission: { reason: blocker },
            motion: { permission: 'HELD', reason: blocker },
          })
    }
    assert.equal((await resumeNavigation()).ok, true)
    assert.deepEqual(requests, [
      'POST /api/v1/navigation/resume',
      'GET /api/v1/navigation/status',
    ])
  })
}

test('control resume does not report success when emergency stop remains active', async () => {
  globalThis.fetch = async (_input, options) => Response.json(options?.method === 'POST'
    ? { ok: true, status: 'motion_resume_acknowledged' }
    : {
        control: { authority: 'NONE', resume_required: false },
        motion: { permission: 'ESTOPPED', reason: 'estop_latched' },
      })
  await assert.rejects(resumeNavigation(), /急停仍在生效/)
})

test('resume rejection does not claim success or poll for it', async () => {
  let requests = 0
  globalThis.fetch = async () => {
    requests += 1
    return Response.json({ ok: false, status: 'stop_confirmation_timeout' })
  }
  await assert.rejects(resumeNavigation(), /stop_confirmation_timeout/)
  assert.equal(requests, 1)
})
