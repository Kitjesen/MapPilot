import assert from 'node:assert/strict'
import test from 'node:test'
import { geographicPoint, parseGeographicPoints, serializeGeographicPoints } from '../src/services/geographicPoints.ts'

const gps = { name: '入口', longitude: 116.391, latitude: 39.907, datum: 'WGS84' as const }
test('GeoJSON imports longitude first and retains WGS84 without applying a display offset', () => {
  assert.deepEqual(parseGeographicPoints(JSON.stringify({ type: 'FeatureCollection', features: [
    { type: 'Feature', properties: { name: '入口' }, geometry: { type: 'Point', coordinates: [116.391, 39.907, 30] } },
  ] })), [gps])
})
test('mixed datum draft round-trips without relabelling AMap points as GPS', () => {
  const points = [gps, { ...gps, name: '高德点', datum: 'GCJ02' as const }]
  assert.deepEqual(parseGeographicPoints(serializeGeographicPoints(points)), points)
})
test('invalid coordinates and unknown datums reject the whole import', () => {
  for (const invalid of [{ longitude: NaN }, { latitude: 91 }, { longitude: '116.391' }, { datum: 'BD09' }, { name: '' }]) {
    assert.throws(() => geographicPoint({ ...gps, ...invalid }))
  }
  assert.throws(() => parseGeographicPoints(JSON.stringify({ version: 1, points: [gps, { ...gps, latitude: 91 }] })))
  assert.throws(() => parseGeographicPoints('{ broken'))
})
test('reject legacy CRS and non-point geometry, limit file size and count', () => {
  assert.throws(() => parseGeographicPoints(JSON.stringify({ type: 'FeatureCollection', crs: 'GCJ02', features: [] })))
  assert.throws(() => parseGeographicPoints(JSON.stringify({ type: 'FeatureCollection', features: [{ type: 'Feature', geometry: { type: 'LineString', coordinates: [] } }] })))
  assert.throws(() => parseGeographicPoints(JSON.stringify({ version: 1, points: Array(501).fill(gps) })))
  assert.throws(() => parseGeographicPoints(' '.repeat(1_000_001)))
  assert.deepEqual(parseGeographicPoints(serializeGeographicPoints([])), [])
})
