import type {
  MapInfo,
  NavigationStatusResponse,
  ProductName,
  SessionEvent,
} from '../types/index.ts'

const NAVIGATION_PRODUCTS = new Set<ProductName>([
  'teleop_avoid',
  'tracking',
  'nav',
  'inspection',
])

const SAVED_MAP_PRODUCTS = new Set<ProductName>([
  'tracking',
  'nav',
  'inspection',
])

export interface ProductWaitOptions {
  fetchSession: () => Promise<SessionEvent['data']>
  fetchNavigation?: () => Promise<NavigationStatusResponse>
  timeoutMs?: number
  intervalMs?: number
  sleep?: (delayMs: number) => Promise<void>
}

export function mapIsActivationReady(
  map: Pick<MapInfo, 'has_pcd' | 'has_octomap' | 'activation_ready'>,
): boolean {
  return map.has_pcd === true
    && map.has_octomap === true
    && map.activation_ready === true
}

export function navigationSessionReady(
  session: SessionEvent['data'],
  mapName: string,
): boolean {
  const activeMap = session.active_map ?? ''
  return session.mode === 'navigating'
    && session.product !== null
    && session.product !== undefined
    && NAVIGATION_PRODUCTS.has(session.product)
    && activeMap === mapName
    && session.map_has_pcd === true
    && session.map_has_octomap === true
    && session.localizer_ready === true
    && session.pose_fresh !== false
}

export function navigationRuntimeReady(
  session: SessionEvent['data'],
  navigation: NavigationStatusResponse,
  mapName: string,
): boolean {
  return navigationSessionReady(session, mapName)
    && navigation.can_accept_goal === true
    && navigation.readiness?.can_accept_goal === true
    && (navigation.readiness?.blockers?.length ?? 0) === 0
}

export function resolveNavigationTargetMapName(
  activeMapName: string | null | undefined,
  selectedMapName: string | null | undefined,
): string | null {
  const selected = selectedMapName?.trim()
  if (selected) return selected
  const active = activeMapName?.trim()
  return active || null
}

export function productReady(
  session: SessionEvent['data'],
  targetProduct: ProductName,
  mapName?: string | null,
): boolean {
  if (session.product !== targetProduct) return false

  if (targetProduct === 'map') return session.mode === 'mapping'
  if (targetProduct === 'explore') {
    if (session.mode !== 'exploring') return false
    const expectedMap = mapName?.trim()
    if (!expectedMap) return !session.active_map
    return session.active_map === expectedMap
      && session.map_has_pcd === true
      && session.map_has_octomap === true
      && session.localizer_ready === true
      && session.pose_fresh !== false
  }
  if (!SAVED_MAP_PRODUCTS.has(targetProduct)) return true

  const expectedMap = mapName?.trim()
  if (!expectedMap) return false
  return navigationSessionReady(session, expectedMap)
}

export function productTransitionDetail(
  session: SessionEvent['data'],
  targetProduct: ProductName,
  mapName?: string | null,
): string {
  if (session.product !== targetProduct) {
    return `等待 Product ${targetProduct}，当前为 ${session.product || 'unknown'}`
  }
  if (mapName && session.active_map !== mapName) {
    return `等待地图 ${mapName}，当前为 ${session.active_map || '未加载'}`
  }
  if (targetProduct === 'map' && session.mode !== 'mapping') return '等待建图会话启动'
  if (targetProduct === 'explore' && session.mode !== 'exploring') return '等待探索会话启动'
  const expectsSavedMap = SAVED_MAP_PRODUCTS.has(targetProduct)
    || (targetProduct === 'explore' && Boolean(mapName?.trim()))
  if (expectsSavedMap) {
    if (!session.map_has_pcd) return '等待保存地图就绪'
    if (!session.localizer_ready) return '等待定位器就绪'
    if (session.pose_fresh === false) return '等待新鲜定位数据'
  }
  if (targetProduct === 'explore' && !mapName?.trim() && session.active_map) {
    return '等待实时探索会话启动'
  }
  return '等待运行时就绪'
}

export async function waitForProductReady(
  targetProduct: ProductName,
  mapName: string | null | undefined,
  options: ProductWaitOptions,
): Promise<SessionEvent['data']> {
  const timeoutMs = options.timeoutMs ?? 90_000
  const intervalMs = options.intervalMs ?? 1_000
  const sleep = options.sleep ?? (delayMs => new Promise(resolve => globalThis.setTimeout(resolve, delayMs)))
  const deadline = Date.now() + timeoutMs
  let lastState = '等待运行时响应'

  while (Date.now() < deadline) {
    let session: SessionEvent['data']
    try {
      session = await options.fetchSession()
    } catch (error) {
      lastState = error instanceof Error ? error.message : String(error)
      await sleep(intervalMs)
      continue
    }

    if (productReady(session, targetProduct, mapName)) {
      if (SAVED_MAP_PRODUCTS.has(targetProduct) && options.fetchNavigation) {
        try {
          const navigation = await options.fetchNavigation()
          if (navigationRuntimeReady(session, navigation, mapName?.trim() ?? '')) {
            return session
          }
          lastState = navigation.readiness?.blockers?.join(', ') || '等待导航输入链路就绪'
        } catch (error) {
          lastState = error instanceof Error ? error.message : String(error)
        }
      } else {
        return session
      }
    } else {
      lastState = productTransitionDetail(session, targetProduct, mapName)
    }
    await sleep(intervalMs)
  }

  throw new Error(`产品模式切换未就绪：${lastState}`)
}
