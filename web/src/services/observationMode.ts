/** A browser observation session reads telemetry without issuing commands. */
export function isObservationMode(search?: string): boolean {
  const query = search ?? (typeof window === 'undefined' ? '' : window.location.search)
  return new URLSearchParams(query).get('observe') === '1'
}

export function dashboardFetch(input: RequestInfo | URL, init?: RequestInit): Promise<Response> {
  const method = (init?.method ?? (input instanceof Request ? input.method : 'GET')).toUpperCase()
  if (isObservationMode() && !['GET', 'HEAD'].includes(method)) {
    return Promise.reject(new Error('只读监控：未发送操作指令'))
  }
  return fetch(input, init)
}
