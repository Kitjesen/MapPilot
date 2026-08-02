import assert from 'node:assert/strict'
import test from 'node:test'

import * as api from '../src/services/api.ts'
import {
  CurrentNavigationTaskStore,
  currentNavigationTaskStore,
  type CurrentNavigationTaskStorage,
} from '../src/services/currentNavigationTask.ts'

class MemoryStorage implements CurrentNavigationTaskStorage {
  readonly values = new Map<string, string>()

  getItem(key: string): string | null {
    return this.values.get(key) ?? null
  }

  setItem(key: string, value: string): void {
    this.values.set(key, value)
  }

  removeItem(key: string): void {
    this.values.delete(key)
  }
}

test('accepted navigation receipt persists task identity separately from request identity', () => {
  const storage = new MemoryStorage()
  const store = new CurrentNavigationTaskStore(storage, () => 123.5)

  const identity = store.trackAccepted({
    schema_version: 1,
    ok: true,
    status: 'submitted',
    task_id: 'task-navigation-42',
    command: {
      name: 'goal',
      task_id: 'task-navigation-42',
      request_id: 'request-attempt-7',
      client_id: 'web-dashboard',
      accepted: true,
      replay: false,
      ts: 123.0,
    },
  })

  assert.deepEqual(identity, {
    schema_version: 'lingtu.web.current_navigation_task.v1',
    task_id: 'task-navigation-42',
    request_id: 'request-attempt-7',
    submitted_at: 123.5,
  })
  assert.deepEqual(store.getSnapshot(), identity)

  const restored = new CurrentNavigationTaskStore(storage, () => 999)
  assert.deepEqual(restored.getSnapshot(), identity)
})

test('accepted navigation receipt without task identity is rejected as a protocol error', () => {
  const store = new CurrentNavigationTaskStore(new MemoryStorage(), () => 123.5)

  assert.throws(
    () => store.trackAccepted({
      schema_version: 1,
      ok: true,
      status: 'submitted',
      command: {
        name: 'goal',
        request_id: 'request-without-task',
        client_id: 'web-dashboard',
        accepted: true,
        replay: false,
        ts: 123.0,
      },
    }),
    /navigation_task_identity_missing/,
  )
  assert.equal(store.getSnapshot(), null)
})

test('task identity subscribers observe replacement and explicit dismissal', () => {
  const storage = new MemoryStorage()
  const store = new CurrentNavigationTaskStore(storage, () => 300)
  const snapshots: Array<string | null> = []
  const unsubscribe = store.subscribe(() => {
    snapshots.push(store.getSnapshot()?.task_id ?? null)
  })

  store.trackAccepted({
    schema_version: 1,
    ok: true,
    status: 'submitted',
    task_id: 'task-observed',
    command: {
      name: 'goal',
      request_id: 'request-observed',
      client_id: 'web-dashboard',
      accepted: true,
      replay: false,
      ts: 299,
    },
  })
  store.clear()
  unsubscribe()

  assert.deepEqual(snapshots, ['task-observed', null])
  assert.equal(store.getSnapshot(), null)
  assert.equal(storage.values.size, 0)
})

test('an empty browser adopts the active task reported by authoritative navigation state', () => {
  const storage = new MemoryStorage()
  const store = new CurrentNavigationTaskStore(storage, () => 350.5)

  const identity = store.adoptAuthoritative({
    task_id: 'task-native-active',
    request_id: 'request-native-active',
  })

  assert.deepEqual(identity, {
    schema_version: 'lingtu.web.current_navigation_task.v1',
    task_id: 'task-native-active',
    request_id: 'request-native-active',
    submitted_at: 350.5,
  })
  assert.deepEqual(store.getSnapshot(), identity)
  assert.deepEqual(
    JSON.parse(storage.values.get('lingtu.current-navigation-task') ?? ''),
    identity,
  )
})

test('authoritative refreshes neither loop on the same task nor clear a retained result', () => {
  const store = new CurrentNavigationTaskStore(new MemoryStorage(), () => 360)
  const adopted = store.adoptAuthoritative({
    task_id: 'task-stable',
    request_id: 'request-start',
  })
  let notifications = 0
  const unsubscribe = store.subscribe(() => {
    notifications += 1
  })

  const same = store.adoptAuthoritative({
    task_id: 'task-stable',
    request_id: 'request-later-control',
  })
  const retained = store.adoptAuthoritative({
    task_id: '',
    request_id: '',
  })
  unsubscribe()

  assert.equal(same, adopted)
  assert.equal(retained, adopted)
  assert.equal(store.getSnapshot(), adopted)
  assert.equal(notifications, 0)
})

test('sendGoal begins tracking the accepted navigation task', async (t) => {
  const originalFetch = globalThis.fetch
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async () => new Response(JSON.stringify({
    schema_version: 1,
    ok: true,
    status: 'submitted',
    task_id: 'task-from-goal-response',
    request_id: 'request-from-goal-response',
    command: {
      name: 'goal',
      task_id: 'task-from-goal-response',
      request_id: 'request-from-goal-response',
      client_id: 'web-dashboard',
      accepted: true,
      replay: false,
      ts: 200,
    },
  }), {
    status: 200,
    headers: { 'Content-Type': 'application/json' },
  })

  await api.sendGoal(1, 2)

  assert.equal(
    currentNavigationTaskStore.getSnapshot()?.task_id,
    'task-from-goal-response',
  )
})

test('navigateClick replaces the tracked task with the accepted task identity', async (t) => {
  const originalFetch = globalThis.fetch
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async () => new Response(JSON.stringify({
    schema_version: 1,
    ok: true,
    status: 'submitted',
    task_id: 'task-from-click-response',
    request_id: 'request-from-click-response',
    command: {
      name: 'navigate_click',
      task_id: 'task-from-click-response',
      request_id: 'request-from-click-response',
      client_id: 'web-dashboard',
      accepted: true,
      replay: false,
      ts: 210,
    },
  }), {
    status: 200,
    headers: { 'Content-Type': 'application/json' },
  })

  await api.navigateClick(3, 4)

  assert.equal(
    currentNavigationTaskStore.getSnapshot()?.task_id,
    'task-from-click-response',
  )
})

test('task status query uses the stable task identity and URL-encodes it', async (t) => {
  const originalFetch = globalThis.fetch
  const requestedUrls: string[] = []
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async (input) => {
    requestedUrls.push(String(input))
    return new Response(JSON.stringify({
      schema_version: 1,
      found: true,
      task_id: 'task/with space',
      request_id: 'request-query',
      status: {
        task_id: 'task/with space',
        request_id: 'request-query',
        state_name: 'PATH_ACTIVE',
        terminal: false,
      },
      reason: '',
      ts: 400,
    }), {
      status: 200,
      headers: { 'Content-Type': 'application/json' },
    })
  }

  const result = await api.fetchNavigationTaskStatus('task/with space')

  assert.equal(requestedUrls[0], '/api/v1/navigation/tasks/task%2Fwith%20space')
  assert.equal(result.status?.state_name, 'PATH_ACTIVE')
})

test('task cancel is addressed by task id and remains a requested action', async (t) => {
  const originalFetch = globalThis.fetch
  let requestedUrl = ''
  let requestedBody: Record<string, unknown> = {}
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async (input, init) => {
    requestedUrl = String(input)
    requestedBody = JSON.parse(String(init?.body)) as Record<string, unknown>
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      status: 'cancel_requested',
      task_id: 'task/cancel me',
      execution_confirmed: false,
      command: {
        name: 'navigation_cancel',
        task_id: 'task/cancel me',
        request_id: 'cancel-attempt',
        client_id: 'web-dashboard',
        accepted: true,
        replay: false,
        ts: 600,
      },
    }), {
      status: 200,
      headers: { 'Content-Type': 'application/json' },
    })
  }

  const result = await api.cancelNavigationTask('task/cancel me')

  assert.equal(requestedUrl, '/api/v1/navigation/tasks/task%2Fcancel%20me/cancel')
  assert.equal(requestedBody.task_id, 'task/cancel me')
  assert.equal(requestedBody.client_id, 'web-dashboard')
  assert.equal(result.status, 'cancel_requested')
  assert.equal(result.execution_confirmed, false)
})

test('task pause addresses the same task with a fresh command request', async (t) => {
  const originalFetch = globalThis.fetch
  let requestedUrl = ''
  let requestedBody: Record<string, unknown> = {}
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async (input, init) => {
    requestedUrl = String(input)
    requestedBody = JSON.parse(String(init?.body)) as Record<string, unknown>
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      status: 'pause_requested',
      task_id: 'task/pause me',
      execution_confirmed: false,
      command: {
        name: 'navigation_task_pause',
        task_id: 'task/pause me',
        request_id: 'pause-attempt',
        client_id: 'web-dashboard',
        accepted: true,
        replay: false,
        ts: 700,
      },
    }), {
      status: 200,
      headers: { 'Content-Type': 'application/json' },
    })
  }

  const before = currentNavigationTaskStore.getSnapshot()
  const result = await api.pauseNavigationTask('task/pause me')

  assert.equal(requestedUrl, '/api/v1/navigation/tasks/task%2Fpause%20me/pause')
  assert.equal(requestedBody.task_id, 'task/pause me')
  assert.equal(requestedBody.client_id, 'web-dashboard')
  assert.match(String(requestedBody.request_id), /^navigation_task_pause-/)
  assert.equal(result.status, 'pause_requested')
  assert.equal(result.execution_confirmed, false)
  assert.equal(currentNavigationTaskStore.getSnapshot(), before)
})
test('task resume keeps task identity and never calls the legacy resume endpoint', async (t) => {
  const originalFetch = globalThis.fetch
  let requestedUrl = ''
  let requestedBody: Record<string, unknown> = {}
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async (input, init) => {
    requestedUrl = String(input)
    requestedBody = JSON.parse(String(init?.body)) as Record<string, unknown>
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      status: 'resume_requested',
      task_id: 'task/resume me',
      execution_confirmed: false,
      command: {
        name: 'navigation_task_resume',
        task_id: 'task/resume me',
        request_id: 'resume-attempt',
        client_id: 'web-dashboard',
        accepted: true,
        replay: false,
        ts: 800,
      },
    }), {
      status: 200,
      headers: { 'Content-Type': 'application/json' },
    })
  }

  const before = currentNavigationTaskStore.getSnapshot()
  const result = await api.resumeNavigationTask('task/resume me')

  assert.equal(requestedUrl, '/api/v1/navigation/tasks/task%2Fresume%20me/resume')
  assert.notEqual(requestedUrl, '/api/v1/navigation/resume')
  assert.equal(requestedBody.task_id, 'task/resume me')
  assert.equal(requestedBody.client_id, 'web-dashboard')
  assert.match(String(requestedBody.request_id), /^navigation_task_resume-/)
  assert.equal(result.status, 'resume_requested')
  assert.equal(result.execution_confirmed, false)
  assert.equal(currentNavigationTaskStore.getSnapshot(), before)
})