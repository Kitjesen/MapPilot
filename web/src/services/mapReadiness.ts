import type {
  MapInfo,
  NavigationStatusResponse,
  ProductModeProfile,
  SessionEvent,
} from '../types/index.ts'

const NAVIGATION_PRODUCT_PROFILES = new Set([
  'teleop_avoid',
  'tracking',
  'nav',
  'inspection',
])

const SAVED_MAP_PRODUCT_PROFILES = new Set<ProductModeProfile>([
  'tracking',
  'nav',
  'inspection',
])

const PRODUCT_SESSIONS: Record<ProductModeProfile, string> = {
  teleop: 'teleop',
  teleop_avoid: 'teleop_avoid',
  map: 'mapping',
  tracking: 'tracking',
  nav: 'navigation',
  inspection: 'inspection',
  tare_explore: 'exploration',
}

export interface ProductProfileWaitOptions {
  fetchSession: () => Promise<SessionEvent['data']>
  fetchNavigation?: () => Promise<NavigationStatusResponse>
  timeoutMs?: number
  intervalMs?: number
  sleep?: (delayMs: number) => Promise<void>
}

export function mapIsNavigationReady(
  map: Pick<MapInfo, 'has_pcd' | 'navigation_ready'>,
): boolean {
  return map.has_pcd === true && map.navigation_ready === true
}

export function navigationSessionReady(
  session: SessionEvent['data'],
  mapName: string,
): boolean {
  const activeMap = session.active_map ?? ''
  return session.mode === 'navigating'
    && session.pending === false
    && NAVIGATION_PRODUCT_PROFILES.has(String(session.product_profile ?? ''))
    && activeMap === mapName
    && session.map_has_pcd === true
    && (session.map_has_octomap === true || session.map_has_tomogram === true)
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

export function productProfileSessionReady(
  session: SessionEvent['data'],
  targetProfile: ProductModeProfile,
  mapName?: string | null,
): boolean {
  if (session.pending || session.error) return false
  if (session.product_profile !== targetProfile) return false

  const productSession = String(session.product_session ?? '')
  if (productSession && productSession !== 'idle' && productSession !== PRODUCT_SESSIONS[targetProfile]) {
    return false
  }

  if (targetProfile === 'map') return session.mode === 'mapping'
  if (targetProfile === 'tare_explore') return session.mode === 'exploring'
  if (!SAVED_MAP_PRODUCT_PROFILES.has(targetProfile)) return true

  const expectedMap = mapName?.trim()
  if (!expectedMap) return false
  return navigationSessionReady(session, expectedMap)
}

export function productProfileTransitionDetail(
  session: SessionEvent['data'],
  targetProfile: ProductModeProfile,
  mapName?: string | null,
): string {
  if (session.pending) return '产品模式正在切换'
  if (session.error) return `产品模式切换失败：${session.error}`
  if (session.product_profile !== targetProfile) {
    return `等待 Profile ${targetProfile}，当前为 ${session.product_profile || 'unknown'}`
  }
  if (mapName && session.active_map !== mapName) {
    return `等待地图 ${mapName}，当前为 ${session.active_map || '未加载'}`
  }
  if (targetProfile === 'map' && session.mode !== 'mapping') return '等待建图会话启动'
  if (targetProfile === 'tare_explore' && session.mode !== 'exploring') return '等待探索会话启动'
  if (SAVED_MAP_PRODUCT_PROFILES.has(targetProfile)) {
    if (!session.localizer_ready) return '等待定位器就绪'
    if (session.pose_fresh === false) return '等待新鲜定位数据'
  }
  return '等待运行时就绪'
}

export async function waitForProductProfileReady(
  targetProfile: ProductModeProfile,
  mapName: string | null | undefined,
  options: ProductProfileWaitOptions,
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

    if (
      session.product_profile === targetProfile
      && !session.pending
      && session.error
    ) {
      throw new Error(`产品模式切换失败：${session.error}`)
    }

    if (productProfileSessionReady(session, targetProfile, mapName)) {
      if (SAVED_MAP_PRODUCT_PROFILES.has(targetProfile) && options.fetchNavigation) {
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
      lastState = productProfileTransitionDetail(session, targetProfile, mapName)
    }
    await sleep(intervalMs)
  }

  throw new Error(`产品模式切换未就绪：${lastState}`)
}
