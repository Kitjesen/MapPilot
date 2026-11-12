import assert from 'node:assert/strict'
import test from 'node:test'
import { numericChannels, traceAt, traceEnd, traceGroups, tracePath, traceSpan, traceTime, type TraceSample } from '../src/services/sensorChart.ts'

const sample = (t: number, value?: number, gap_before = false): TraceSample => ({ t, values: { value }, gap_before })

test('inspection returns real samples and leaves explicit gaps empty', () => {
  const rows = [sample(1,0),sample(2,4),sample(5,9,true),sample(6,8)]
  assert.equal(traceAt(rows,0), undefined)
  assert.equal(traceAt(rows,7), undefined)
  assert.equal(traceAt(rows,1.2)?.values.value, 0)
  assert.equal(traceAt(rows,4), undefined)
  assert.equal(traceAt(rows,5)?.values.value, 9)
  assert.equal(traceAt(rows,6)?.values.value, 8)
})

test('time zoom and seeking stay within retained samples', () => {
  const rows = [sample(20,1),sample(100,2)]
  assert.equal(traceEnd(rows,null),100)
  assert.equal(traceEnd(rows,10),20)
  assert.equal(traceEnd(rows,150),100)
  assert.equal(traceEnd(rows,50),50)
  assert.equal(traceSpan(2),5)
  assert.equal(traceSpan(301),300)
  assert.equal(traceSpan(42.5),43)
  assert.equal(traceTime(12.25,undefined,true),'12.250 s')
  assert.match(traceTime(1.25,1700000000,true), /^\d{2}:\d{2}:\d{2}\.250$/)
})

test('channels retain identifiers and different physical units never share a plot', () => {
  const channels = numericChannels(['joint0 · 位置 (rad)','joint1 · 位置 (rad)','joint0 · 力矩 (N·m)','通道 0 · V'])
  const groups = traceGroups(channels)
  assert.equal(groups.length,3)
  assert.equal(groups[0][1].length,2)
  assert.deepEqual(groups.map(([,items]) => items[0].unit),['rad','N·m','V'])
  assert.equal(channels[0].id,'joint0 · 位置 (rad)')
})

test('zoomed plots keep spikes and do not bridge missing readings', () => {
  const rows = [sample(0,0),sample(.001,10),sample(.002,0),sample(.003),sample(.004,2),sample(.005,3)]
  const path = tracePath(rows,'value',0,300,0,10)
  assert.ok(path.includes('10.0'))
  assert.equal((path.match(/M/g) ?? []).length,2)
  assert.equal(tracePath(rows,'unknown',0,300,0,10),'')
  assert.ok(!path.includes('NaN'))
})
