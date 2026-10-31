export interface MapPreview {
  buffer: ArrayBuffer
  mapId: string
  contentEpoch: number
  frameId: string
}

function requiredHeader(response: Response, name: string): string {
  const value = response.headers.get(name)?.trim()
  if (!value) throw new Error(`地图预览响应缺少 ${name}`)
  return value
}

export async function fetchMapPreview(
  mapName: string,
  expectedContentEpoch?: number | null,
  signal?: AbortSignal,
  artifact: 'pcd' | 'octomap' = 'pcd',
): Promise<MapPreview> {
  const response = await fetch(`/api/v1/maps/${encodeURIComponent(mapName)}/${artifact}`, {
    cache: 'no-store',
    signal,
  })
  if (!response.ok) throw new Error(`地图${artifact === 'octomap' ? '体素' : '点云'}读取失败（HTTP ${response.status}）`)

  const mapId = requiredHeader(response, 'X-Lingtu-Map-Id')
  const epochText = requiredHeader(response, 'X-Lingtu-Content-Epoch')
  const frameId = requiredHeader(response, 'X-Lingtu-Frame-Id')
  const contentEpoch = Number(epochText)
  if (mapId !== mapName) throw new Error('地图预览已切换，请刷新后重试')
  if (!Number.isSafeInteger(contentEpoch) || contentEpoch <= 0) {
    throw new Error('地图预览响应包含无效的内容版本')
  }
  if (expectedContentEpoch != null && contentEpoch !== expectedContentEpoch) {
    throw new Error('地图内容已更新，请刷新地图库后重试')
  }

  return { buffer: await response.arrayBuffer(), mapId, contentEpoch, frameId }
}
