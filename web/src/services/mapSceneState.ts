import type { MapSceneEvent, MapSceneLayer } from '../types'

const ELEVATION_MAX_CELLS = 131_072

function elevationLayer(scene: MapSceneEvent | null | undefined): MapSceneLayer | undefined {
  return scene?.layers.find(layer => (
    layer.id === 'maps.elevation'
    || layer.topic === '/maps/elevation'
  ))
}

function rasterInteger(
  scene: MapSceneEvent,
  layer: MapSceneLayer | undefined,
  key: 'generation' | 'reset_epoch',
): number | null {
  const value = layer?.[key] ?? scene.metadata?.[key]
  return typeof value === 'number' && Number.isInteger(value) && value >= 0 ? value : null
}

function sceneSequence(scene: MapSceneEvent): number | null {
  return typeof scene.sequence === 'number' && Number.isInteger(scene.sequence) && scene.sequence >= 0
    ? scene.sequence
    : null
}

function sceneMapId(scene: MapSceneEvent): string | null {
  return typeof scene.map_id === 'string' && scene.map_id.trim() ? scene.map_id.trim() : null
}

function hasInlineElevation(layer: MapSceneLayer | undefined): boolean {
  return typeof layer?.grid_b64 === 'string' && layer.grid_b64.length > 0
}

function nonNegativeInteger(value: unknown): number | null {
  return typeof value === 'number' && Number.isInteger(value) && value >= 0 ? value : null
}

function finiteNumber(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null
}

function elevationCohort(scene: MapSceneEvent, layer: MapSceneLayer | undefined): string | null {
  if (!layer) return null
  const producerBootId = String(
    layer.producer_boot_id ?? scene.metadata?.producer_boot_id ?? '',
  ).trim()
  const frameId = String(layer.frame_id ?? scene.frame_id ?? '').trim()
  const resetEpoch = nonNegativeInteger(layer.reset_epoch ?? scene.metadata?.reset_epoch)
  const rows = nonNegativeInteger(layer.rows)
  const cols = nonNegativeInteger(layer.cols)
  const resolution = finiteNumber(layer.resolution)
  const yaw = finiteNumber(layer.yaw)
  const downsampleFactor = nonNegativeInteger(layer.downsample_factor)
  const origin = layer.origin
  if (!producerBootId || !frameId || resetEpoch === null || rows === null || cols === null
    || rows <= 0 || cols <= 0 || resolution === null || resolution <= 0 || yaw === null
    || downsampleFactor === null || downsampleFactor <= 0 || !Array.isArray(origin)
    || origin.length < 3 || !origin.slice(0, 3).every(value => finiteNumber(value) !== null)) return null
  return JSON.stringify([
    producerBootId,
    frameId,
    resetEpoch,
    sceneMapId(scene),
    rows,
    cols,
    resolution,
    origin.slice(0, 3),
    yaw,
    downsampleFactor,
  ])
}

function rejectsRetainedElevation(layer: MapSceneLayer | undefined): boolean {
  if (!layer) return false
  if (layer.valid === false || layer.oversize === true || layer.retain_previous === false) return true
  if (layer.valid_count === 0) return true
  const status = `${String(layer.payload ?? '')} ${String(layer.reason ?? '')}`.toLowerCase()
  if (/invalid|unsafe|oversize|too[_ -]?large|capacity|malformed|reject/.test(status)) return true
  const rows = layer.rows
  const cols = layer.cols
  return typeof rows === 'number' && typeof cols === 'number'
    && (!Number.isSafeInteger(rows * cols) || rows * cols <= 0 || rows * cols > ELEVATION_MAX_CELLS)
}

/** Preserve one aligned inline elevation raster across higher-rate metadata events. */
export function mergeMapSceneElevation(
  previous: MapSceneEvent | null | undefined,
  incoming: MapSceneEvent,
): MapSceneEvent {
  if (!previous) return incoming
  const priorElevation = elevationLayer(previous)
  const nextElevation = elevationLayer(incoming)
  if (!priorElevation || !hasInlineElevation(priorElevation) || !nextElevation) return incoming
  if (rejectsRetainedElevation(nextElevation)) return incoming
  const previousCohort = elevationCohort(previous, priorElevation)
  const incomingCohort = elevationCohort(incoming, nextElevation)
  if (!previousCohort || previousCohort !== incomingCohort) return incoming

  const previousSequence = sceneSequence(previous)
  const incomingSequence = sceneSequence(incoming)
  const previousGeneration = rasterInteger(previous, priorElevation, 'generation')
  const incomingGeneration = rasterInteger(incoming, nextElevation, 'generation')
  if (previousSequence === null || incomingSequence === null
    || previousGeneration === null || incomingGeneration === null) return incoming
  if (incomingSequence < previousSequence || incomingGeneration < previousGeneration) return previous
  if (hasInlineElevation(nextElevation)) return incoming
  if (nextElevation.retain_previous !== true
    || nextElevation.retention_scope !== 'same_elevation_cohort') return incoming
  const payloadGeneration = nonNegativeInteger(
    priorElevation.payload_generation ?? priorElevation.generation,
  )
  const payloadObservationSequence = nonNegativeInteger(
    priorElevation.payload_observation_sequence ?? priorElevation.observation_sequence,
  )
  const payloadStamp = finiteNumber(priorElevation.payload_stamp_s ?? priorElevation.stamp_s)
  const retained = {
    ...nextElevation,
    ...priorElevation,
    payload: 'retained',
    reason: nextElevation.reason,
    retain_previous: true,
    retention_scope: 'same_elevation_cohort',
    payload_retained: true,
    payload_generation: payloadGeneration,
    payload_observation_sequence: payloadObservationSequence,
    payload_stamp_s: payloadStamp,
    retained_for_sequence: incoming.sequence,
    retained_for_generation: incomingGeneration,
  }
  const layers = [...incoming.layers]
  layers[layers.indexOf(nextElevation)] = retained
  return { ...incoming, layers }
}
