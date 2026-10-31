import { useCallback, useEffect, useState } from 'react'
import { RecordingPanel } from '../src/components/RecordingPanel'
import { fetchRecordingStatus } from '../src/services/api'
import { RECORDING_STATUS_POLL_MS } from '../src/services/recordingStatus'
import type { RecordingStatusResponse } from '../src/types'

export function Preview() {
  const [status, setStatus] = useState<RecordingStatusResponse | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [notice, setNotice] = useState('')
  const [embedded, setEmbedded] = useState(false)
  const [dark, setDark] = useState(false)
  const refresh = useCallback(() => fetchRecordingStatus().then(next => {
      setStatus(next)
      setError(null)
    }).catch((cause: unknown) => {
      setError(cause instanceof Error ? cause.message : String(cause))
    }), [])
  useEffect(() => {
    void refresh()
    const timer = window.setInterval(() => void refresh(), RECORDING_STATUS_POLL_MS)
    return () => window.clearInterval(timer)
  }, [refresh])
  const toast = (message: string) => { setNotice(message) }
  return <main style={{ maxWidth: 1100, margin: '24px auto', padding: '0 20px' }}>
    <header style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 12, flexWrap: 'wrap' }}>
      <h1 style={{ fontSize: 18, fontWeight: 500 }}>数据工作区 · 隔离验证</h1>
      <div style={{ display: 'flex', gap: 8 }}>
        <button style={{ border: '1px solid var(--border)', background: 'var(--bg-layer-1)', color: 'var(--text)', borderRadius: 8, padding: '6px 10px' }} onClick={() => setEmbedded(value => !value)}>{embedded ? '工作区布局' : '侧栏布局'}</button>
        <button style={{ border: '1px solid var(--border)', background: 'var(--bg-layer-1)', color: 'var(--text)', borderRadius: 8, padding: '6px 10px' }} onClick={() => { setDark(value => !value); document.documentElement.dataset.theme = dark ? 'light' : 'dark' }}>{dark ? '浅色' : '深色'}</button>
      </div>
    </header>
    <p style={{ color: 'var(--text-dim)', fontSize: 14, lineHeight: 1.6 }}>使用真实 RecordingPanel 和 Gateway 录制接口。此页面不启动导航或机器人运动；本地夹具数据不能代表实机采集结果。</p>
    {notice && <p role="status" style={{ padding: 12, border: '1px solid var(--border)', borderRadius: 10 }}>{notice}</p>}
    <div style={{ maxWidth: embedded ? 420 : undefined, margin: '0 auto', padding: embedded ? 16 : 0, background: 'var(--bg-layer-1)', border: '1px solid var(--border)', borderRadius: 16 }}>
      <RecordingPanel embedded={embedded} locale="zh" status={status} statusError={error} refreshStatus={refresh} showToast={toast} />
    </div>
  </main>
}

