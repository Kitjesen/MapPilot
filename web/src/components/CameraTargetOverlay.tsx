import type { SceneTargetPresentation } from '../services/sceneTarget'
import styles from './CameraTargetOverlay.module.css'

interface CameraTargetOverlayProps {
  target: SceneTargetPresentation | null
}

/** A status caption: no pixel lock without a bbox aligned to the shown frame. */
export function CameraTargetOverlay({ target }: CameraTargetOverlayProps) {
  if (!target?.cameraVisible) return null
  return (
    <div className={styles.caption} data-state={target.state} role="status" title={target.detail}>
      <span className={styles.label}>{target.label}</span>
      {target.targetLabel && <span className={styles.target}>{target.targetLabel}</span>}
    </div>
  )
}
