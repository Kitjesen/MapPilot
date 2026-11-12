import { useCallback, useEffect, useState, type MouseEvent } from 'react'
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
  const [view, setView] = useState<'record' | 'history'>('record')
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
  const blockDemoDownload = (event: MouseEvent<HTMLElement>) => {
    if ((event.target as Element).closest('a[href^="/api/v1/recordings/"]')) {
      event.preventDefault()
      toast('演示记录没有真实文件，无法下载。')
    }
  }
  return <main style={{ maxWidth: 800, margin: '24px auto', padding: '0 16px' }} onClick={blockDemoDownload} onAuxClick={blockDemoDownload}>
    <header style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: 12, flexWrap: 'wrap' }}>
      <h1 style={{ fontSize: 16, fontWeight: 500 }}>演示数据 · 不连接机器人</h1>
      <div style={{ display: 'flex', gap: 8 }}>
        <button style={{ border: '1px solid var(--border)', background: 'var(--bg-layer-1)', color: 'var(--text)', borderRadius: 8, padding: '6px 10px' }} onClick={() => setEmbedded(value => !value)}>{embedded ? '工作区布局' : '侧栏布局'}</button>
        <button style={{ border: '1px solid var(--border)', background: 'var(--bg-layer-1)', color: 'var(--text)', borderRadius: 8, padding: '6px 10px' }} onClick={() => { setDark(value => !value); document.documentElement.dataset.theme = dark ? 'light' : 'dark' }}>{dark ? '浅色' : '深色'}</button>
      </div>
    </header>
    {notice && <p role="status" style={{ padding: 12, border: '1px solid var(--border)', borderRadius: 10 }}>{notice}</p>}
    <div style={{ maxWidth: embedded ? 420 : undefined, margin: '0 auto', padding: embedded ? 12 : 0 }}>
      <RecordingPanel view={view} onViewChange={setView} embedded={embedded} locale="zh" status={status} statusError={error} refreshStatus={refresh} showToast={toast} />
    </div>
  </main>
}

