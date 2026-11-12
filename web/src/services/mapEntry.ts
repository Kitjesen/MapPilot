import type { Tab } from '../types/index.ts'

/** URL parameters only select a page; they never activate a map or Product. */
export function initialMapEntry(search: string): { tab: Tab; map: string | null } {
  const query = new URLSearchParams(search)
  const map = query.get('map')?.trim() || null
  const tab = query.get('tab')
  return { tab: tab === 'map' || map !== null ? 'map'
    : tab === 'mapping' ? tab : 'scene', map }
}
