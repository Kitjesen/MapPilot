import test from 'node:test'
import assert from 'node:assert/strict'
import { findInspectionPoints, routePointFromLocation } from '../src/services/inspectionPoints.ts'
import type { LocationEntry } from '../src/types/index.ts'

const points: LocationEntry[] = [
  { name: '一号配电柜', x: 1, y: 2, z: 1.3, yaw: .5, tags: ['inspection'], metadata: {
    region: '东区机房', inspection: { dwell: 7, action: 'capture:indicator_green' } } },
  { name: '二号配电柜', x: 3, y: 4, z: 1.3, tags: [], metadata: { region: '东区机房' } },
  { name: '北门', x: 4, y: 2, z: 1.3, tags: [], metadata: { region: '入口' } },
]

test('region lookup returns all candidates, while point names select the matching entry', () => {
  assert.deepEqual(findInspectionPoints(points, '东区').map(p => p.name), ['一号配电柜', '二号配电柜'])
  assert.deepEqual(findInspectionPoints(points, ' 一号 ').map(p => p.name), ['一号配电柜'])
  assert.deepEqual(findInspectionPoints(points, '不存在'), [])
})

test('route creation uses the saved point action, dwell, body height and heading', () => {
  assert.deepEqual(routePointFromLocation(points[0]), { id: '一号配电柜', x: 1, y: 2, z: 1.3, yaw: .5,
    tolerance: .35, dwell: 7, action: 'capture:indicator_green', enabled: true })
  assert.equal(routePointFromLocation(points[1]).action, 'capture:overview')
  assert.equal(routePointFromLocation({ ...points[0], metadata: { inspection: { dwell: 0, action: '' } } }).action, '')
})
