import { dashboardFetch } from './observationMode.ts'
import type { LocalizationInitialPose } from './localizationInitialPose.ts'

export interface ProductSwitchRequest {
  request_id: string
  product: 'map' | 'nav'
  map_name: string | null
  expected_product_session_id: string
  initial_pose?: LocalizationInitialPose
}

export interface ProductSwitchOperation {
  request_id: string
  request: ProductSwitchRequest
  state: 'running' | 'succeeded' | 'failed' | 'interrupted'
  message: string
  result: { product_session_id?: string; ok?: boolean } | null
}

export interface ProductControlSnapshot {
  available: boolean
  robot: string
  env: 'real' | 'sim'
  current: { product: string | null; product_session_id?: string; status: string }
  operation: ProductSwitchOperation | null
}

export class ProductControlError extends Error {
  readonly status: number
  constructor(message: string, status: number) { super(message); this.status = status }
}

async function controlFetch<T>(path: string, body?: ProductSwitchRequest): Promise<T> {
  const response = await dashboardFetch(`/api/v1/product-control${path}`, {
    method: body ? 'POST' : 'GET',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
    signal: AbortSignal.timeout(5_000),
  })
  const result = await response.json()
  if (!response.ok) throw new ProductControlError(result.message || '模式切换服务未连接', response.status)
  return result as T
}

export const fetchProductControl = () => controlFetch<ProductControlSnapshot>('')
export const fetchProductOperation = (id: string) =>
  controlFetch<ProductSwitchOperation>(`/operations/${encodeURIComponent(id)}`)
export const submitProductSwitch = (request: ProductSwitchRequest) =>
  controlFetch<ProductSwitchOperation>('/switch', request)

export function switchIsTerminal(operation: ProductSwitchOperation): boolean {
  return operation.state !== 'running'
}
