/** URL parameters only select a page; they never activate a map or Product. */
export function initialMapEntry(search: string): { tab: 'map' | 'scene'; map: string | null } {
  const query = new URLSearchParams(search)
  const map = query.get('map')?.trim() || null
  return { tab: query.get('tab') === 'map' || map !== null ? 'map' : 'scene', map }
}
