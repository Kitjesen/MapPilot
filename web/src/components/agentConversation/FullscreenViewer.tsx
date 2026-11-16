import { useEffect, useRef } from 'react'
import { Maximize2, X } from 'lucide-react'
import styles from './FullscreenViewer.module.css'

export type FullscreenTable = {
  columns: string[]
  rows: Array<Array<string | number | null>>
}

export type FullscreenViewerProps = {
  open: boolean
  title: string
  kind: 'image' | 'video' | 'table' | 'map'
  src?: string
  alt?: string
  table?: FullscreenTable
  children?: React.ReactNode
  onClose: () => void
}

export function FullscreenViewer({ open, title, kind, src, alt = '', table, children, onClose }: FullscreenViewerProps) {
  const closeButton = useRef<HTMLButtonElement>(null)

  useEffect(() => {
    if (!open) return
    const previousOverflow = document.body.style.overflow
    document.body.style.overflow = 'hidden'
    closeButton.current?.focus()
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') onClose()
    }
    document.addEventListener('keydown', onKeyDown)
    return () => {
      document.body.style.overflow = previousOverflow
      document.removeEventListener('keydown', onKeyDown)
    }
  }, [open, onClose])

  if (!open) return null

  return <div className={styles.backdrop} role="dialog" aria-modal="true" aria-label={title}
    onMouseDown={event => { if (event.target === event.currentTarget) onClose() }}>
    <section className={styles.viewer}>
      <header className={styles.header}>
        <div className={styles.title}><Maximize2 size={15} aria-hidden="true" /><strong>{title}</strong><span>{kindLabel(kind)}</span></div>
        <button ref={closeButton} type="button" className={styles.close} onClick={onClose} aria-label="关闭全屏查看器"><X size={18} /></button>
      </header>
      <div className={`${styles.content} ${styles[kind]}`}>
        {kind === 'image' && src && <img src={src} alt={alt || title} />}
        {kind === 'video' && src && <video src={src} controls playsInline preload="metadata" aria-label={alt || title} />}
        {kind === 'table' && <Table table={table} title={title} />}
        {kind === 'map' && children}
        {kind !== 'map' && !src && kind !== 'table' && <p className={styles.empty}>当前内容没有可查看的地址。</p>}
        {kind === 'map' && !children && <p className={styles.empty}>当前地图没有可查看的内容。</p>}
      </div>
    </section>
  </div>
}

function Table({ table, title }: { table?: FullscreenTable; title: string }) {
  if (!table) return <p className={styles.empty}>当前表格没有可查看的数据。</p>
  return <div className={styles.tableWrap} role="region" aria-label={`${title}表格`} tabIndex={0}>
    <table>
      <thead><tr>{table.columns.map((column, index) => <th key={`${column}-${index}`} scope="col">{column}</th>)}</tr></thead>
      <tbody>{table.rows.map((row, rowIndex) => <tr key={rowIndex}>{table.columns.map((_, columnIndex) => <td key={columnIndex}>{formatCell(row[columnIndex])}</td>)}</tr>)}</tbody>
    </table>
  </div>
}

function formatCell(value: string | number | null | undefined) {
  return value == null ? '—' : String(value)
}

function kindLabel(kind: FullscreenViewerProps['kind']) {
  return { image: '图片', video: '视频', table: '表格', map: '地图' }[kind]
}
