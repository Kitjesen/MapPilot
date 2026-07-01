import type { Tab } from '../types'
import styles from './TabBar.module.css'

interface TabBarProps {
  activeTab: Tab
  onTabChange: (tab: Tab) => void
}

const TABS: { key: Tab; label: string }[] = [
  { key: 'console', label: '控制台' },
  { key: 'scene',   label: '场景' },
  { key: 'map',     label: '地图' },
  { key: 'slam',    label: 'SLAM' },
]

export function TabBar({ activeTab, onTabChange }: TabBarProps) {
  const handleKeyDown = (e: React.KeyboardEvent, idx: number) => {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return
    e.preventDefault()
    const delta = e.key === 'ArrowRight' ? 1 : -1
    const next = (idx + delta + TABS.length) % TABS.length
    onTabChange(TABS[next].key)
  }

  return (
    <nav className={styles.tabBar} role="tablist" aria-label="主视图切换">
      {TABS.map((tab, i) => (
        <button
          key={tab.key}
          role="tab"
          tabIndex={activeTab === tab.key ? 0 : -1}
          aria-selected={activeTab === tab.key}
          aria-controls={`panel-${tab.key}`}
          className={activeTab === tab.key ? styles.tabBtnActive : styles.tabBtn}
          onClick={() => onTabChange(tab.key)}
          onKeyDown={(e) => handleKeyDown(e, i)}
        >
          {tab.label}
        </button>
      ))}
    </nav>
  )
}
