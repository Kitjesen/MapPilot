import { useCallback, useEffect, useMemo, useRef, useState, type PointerEvent } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import type { Octomap } from '../services/octomap.ts'
import type { PcdCloud } from '../services/pcdPreview.ts'
import type { NoGoRegion } from '../services/noGo.ts'
import type { Bounds, XYZ } from '../types/stairs.ts'
import type { LocationEntry } from '../types/index.ts'
import { pickVisiblePcdPoint } from '../services/pcdPicking.ts'
import type { MapEditOperation, MapEditPreview, MapEditScene, MapEditView } from '../types/mapEdit.ts'
import { applyMapEditOperations, isLocalMapEdit, mapEditCloudBounds, mapEditKeepsPoint, planePoints, pointInMapEditBounds } from '../services/mapEdit.ts'
import { lingtuToThree } from '../services/coordinateFrame.ts'
import { createSavedMapLayer, SAVED_MAP_Z_CEIL, SAVED_MAP_Z_FLOOR } from './scene3d/layers/savedMapLayer.ts'
import { createOctomapLayer } from './scene3d/layers/octomapLayer.ts'
import { createPcdLayer } from './scene3d/layers/pcdLayer.ts'
import { createNoGoLayer } from './scene3d/layers/noGoLayer.ts'
import { selectSavedMapBounds, type ScreenRectangle } from './scene3d/stairAnnotations.ts'
import { createMapEditBox, frameMapEditView, mapEditSelectionAnchor } from './scene3d/mapEditing.ts'
import styles from './MapGallery.module.css'

interface Props {
  name: string
  points?: number[]
  cloud?: PcdCloud
  octomap?: Octomap
  voxelScale: number
  pointSize: number
  opacity: number
  ceiling: number | null
  grid: boolean
  gridStep: number
  noGoRegions?: readonly NoGoRegion[]
  mapEdit?: MapEditScene | null
  onMapEditBounds?: (bounds: Bounds) => void
  onMapEditAnchor?: (anchor: { x: number; y: number } | null) => void
  onReady?: (handle: MapGallerySceneHandle | null) => void
  pointPicking?: boolean
  onPointPick?: (point: XYZ) => void
  inspectionPoints?: readonly LocationEntry[]
}

export interface MapGallerySceneHandle { mapEditView(view: MapEditView): void; focusPoint(point: XYZ): void; snapshot(): string | undefined; resetView(): void }

type MapLayer = THREE.Points<THREE.BufferGeometry, THREE.PointsMaterial | THREE.ShaderMaterial>
  | THREE.InstancedMesh<THREE.BoxGeometry, THREE.MeshBasicMaterial>

function previewDistance(bounds: THREE.Box3, aspect: number, direction: THREE.Vector3): number {
  const center = bounds.getCenter(new THREE.Vector3())
  const right = new THREE.Vector3().crossVectors(new THREE.Vector3(0, 1, 0), direction).normalize()
  const up = new THREE.Vector3().crossVectors(direction, right)
  const tangent = Math.tan(Math.PI / 8)
  let distance = .1
  for (const x of [bounds.min.x, bounds.max.x]) for (const y of [bounds.min.y, bounds.max.y]) for (const z of [bounds.min.z, bounds.max.z]) {
    const offset = new THREE.Vector3(x, y, z).sub(center)
    distance = Math.max(distance, offset.dot(direction) + Math.max(Math.abs(offset.dot(right)) / (tangent * aspect), Math.abs(offset.dot(up)) / tangent))
  }
  return distance * 1.02
}

function projectAxes(rotation: THREE.Quaternion) {
  const inverse = rotation.clone().invert()
  return [
    { name: 'X', color: '#cf5968', direction: [1, 0, 0] as const },
    { name: 'Y', color: '#4d9878', direction: [0, 1, 0] as const },
    { name: 'Z', color: '#587fc0', direction: [0, 0, 1] as const },
  ].map(axis => {
    const point = new THREE.Vector3(...lingtuToThree(axis.direction)).applyQuaternion(inverse)
    return { name: axis.name, color: axis.color, x: 44 + point.x * 27, y: 44 - point.y * 27, depth: point.z }
  }).sort((a, b) => a.depth - b.depth)
}

function drawAxes(context: CanvasRenderingContext2D, rotation: THREE.Quaternion) {
  context.clearRect(0, 0, 88, 88)
  context.lineWidth = 1.3
  context.font = '600 11px sans-serif'
  context.textAlign = 'center'
  context.textBaseline = 'middle'
  for (const axis of projectAxes(rotation)) {
    context.globalAlpha = .8 + axis.depth * .2
    context.strokeStyle = context.fillStyle = axis.color
    context.beginPath()
    context.moveTo(44, 44)
    context.lineTo(44 + (axis.x - 44) * .75, 44 + (axis.y - 44) * .75)
    context.stroke()
    context.fillText(axis.name, axis.x, axis.y + .5)
  }
  context.globalAlpha = 1
}

function editedCloud(cloud: PcdCloud | undefined, editView?: string, operations?: readonly MapEditOperation[], preview?: MapEditPreview | null): PcdCloud | undefined {
  if (!cloud || editView !== 'draft' || !operations?.length) return cloud
  const positions = operations.every(isLocalMapEdit)
    ? applyMapEditOperations(cloud.positions, operations) : preview?.preview_points ?? []
  let bounds = cloud.bounds
  for (const operation of operations) if (operation.operation === 'add_plane') {
    if (bounds === cloud.bounds) bounds = { min: [...bounds.min], max: [...bounds.max] }
    for (const [index, axis] of (['x', 'y', 'z'] as const).entries()) {
      bounds.min[index] = Math.min(bounds.min[index], axis === operation.axis ? operation.offset : operation.bounds.min[axis])
      bounds.max[index] = Math.max(bounds.max[index], axis === operation.axis ? operation.offset : operation.bounds.max[axis])
    }
  }
  return { ...cloud, positions: new Float32Array(positions), bounds }
}

export function MapGalleryScene({ name, points, cloud, octomap, voxelScale, pointSize, opacity, ceiling, grid, gridStep,
  noGoRegions, mapEdit = null, onMapEditBounds, onMapEditAnchor, onReady, pointPicking, onPointPick, inspectionPoints }: Props) {
  const mount = useRef<HTMLDivElement>(null)
  const axes = useRef<HTMLCanvasElement>(null)
  const view = useRef<{
    scene: THREE.Scene; camera: THREE.PerspectiveCamera; renderer: THREE.WebGLRenderer
    controls: OrbitControls; bounds: THREE.Box3; fitted: boolean; render(): void
  } | null>(null)
  const layer = useRef<MapLayer | null>(null)
  const selectionRectangle = useRef<HTMLDivElement>(null)
  const pointer = useRef<{ id: number; rectangle: ScreenRectangle } | null>(null)
  const pointPointer = useRef<{ x: number; y: number } | null>(null)
  const editRef = useRef(mapEdit)
  const anchorCallback = useRef(onMapEditAnchor)
  const lastAnchor = useRef<{ x: number; y: number } | null>(null)
  const editView = mapEdit?.view
  const editOperations = mapEdit?.operations
  const editPreview = mapEdit?.preview
  const displayCloud = useMemo(() => editedCloud(cloud, editView, editOperations, editPreview),
    [cloud, editView, editOperations, editPreview])
  const [error, setError] = useState('')

  useEffect(() => { editRef.current = mapEdit; anchorCallback.current = onMapEditAnchor }, [mapEdit, onMapEditAnchor])

  const fit = useCallback(() => {
    const v = view.current
    if (!v || v.bounds.isEmpty()) return
    const damping = v.controls.enableDamping
    v.controls.enableDamping = false
    v.controls.update()
    v.controls.enableDamping = damping
    const center = v.bounds.getCenter(new THREE.Vector3())
    const radius = v.bounds.getSize(new THREE.Vector3()).length() / 2
    const direction = new THREE.Vector3(1, 1.5, 1.4).normalize()
    const distance = previewDistance(v.bounds, v.camera.aspect, direction)
    v.controls.minDistance = Math.max(.05, radius * .03)
    v.controls.maxDistance = distance * 8
    v.camera.far = distance * 20
    v.camera.updateProjectionMatrix()
    v.camera.position.copy(center).addScaledVector(direction, distance)
    v.controls.target.copy(center)
    v.controls.update()
    v.render()
  }, [])

  const zoom = useCallback((scale: number) => {
    const v = view.current
    if (!v) return
    const offset = v.camera.position.clone().sub(v.controls.target)
    offset.setLength(THREE.MathUtils.clamp(offset.length() * scale, v.controls.minDistance, v.controls.maxDistance))
    v.camera.position.copy(v.controls.target).add(offset)
    v.controls.update()
    v.render()
  }, [])

  useEffect(() => {
    const element = mount.current!
    let renderer: THREE.WebGLRenderer
    try { renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' }) }
    catch (cause) {
      // eslint-disable-next-line react-hooks/set-state-in-effect -- WebGL initialization can fail only after the canvas mounts.
      setError(cause instanceof Error ? cause.message : '浏览器无法显示三维地图')
      return
    }
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
    renderer.localClippingEnabled = true
    renderer.domElement.tabIndex = 0
    renderer.domElement.setAttribute('aria-label', `${name} 交互地图`)
    renderer.domElement.setAttribute('aria-description', '拖拽旋转，滚轮缩放；方向键平移，+ 和 - 缩放，Home 复位。')
    element.appendChild(renderer.domElement)
    const scene = new THREE.Scene()
    const camera = new THREE.PerspectiveCamera(45, 1, .01, 1000)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.maxPolarAngle = Math.PI * .95
    controls.enableDamping = !matchMedia('(prefers-reduced-motion: reduce)').matches
    controls.dampingFactor = .1
    const ratio = Math.min(devicePixelRatio, 2)
    const axesCanvas = axes.current!
    axesCanvas.width = axesCanvas.height = 88 * ratio
    const axesContext = axesCanvas.getContext('2d')!
    axesContext.scale(ratio, ratio)
    let frame = 0
    const render = () => {
      if (frame) return
      frame = requestAnimationFrame(() => {
        frame = 0
        controls.update()
        renderer.render(scene, camera)
        drawAxes(axesContext, camera.quaternion)
        if (anchorCallback.current) {
          const selection = editRef.current?.selection
          const anchor = selection ? mapEditSelectionAnchor(camera,
            { width: element.clientWidth, height: element.clientHeight }, selection) : null
          if (anchor?.x !== lastAnchor.current?.x || anchor?.y !== lastAnchor.current?.y) {
            lastAnchor.current = anchor
            anchorCallback.current(anchor)
          }
        }
      })
    }
    controls.addEventListener('change', render)
    controls.listenToKeyEvents(renderer.domElement)
    view.current = { scene, camera, renderer, controls, bounds: new THREE.Box3(), fitted: false, render }
    const resize = () => {
      const width = Math.max(1, element.clientWidth), height = Math.max(1, element.clientHeight)
      const aspect = width / height
      const v = view.current!
      if (v.fitted) {
        // Preserve the chosen view and relative zoom when the panel or window changes size.
        const offset = camera.position.clone().sub(controls.target)
        const direction = offset.clone().normalize()
        const distance = previewDistance(v.bounds, aspect, direction)
        offset.multiplyScalar(distance / previewDistance(v.bounds, camera.aspect, direction))
        camera.position.copy(controls.target).add(offset)
        controls.maxDistance = distance * 8
        camera.far = distance * 20
        controls.update()
      }
      camera.aspect = aspect
      camera.updateProjectionMatrix()
      renderer.setSize(width, height)
      render()
    }
    const observer = new ResizeObserver(resize)
    observer.observe(element)
    resize()
    return () => {
      observer.disconnect()
      cancelAnimationFrame(frame)
      controls.removeEventListener('change', render)
      controls.dispose()
      renderer.dispose()
      renderer.domElement.remove()
      view.current = null
      pointer.current = null
    }
  }, [name])

  useEffect(() => {
    onReady?.({ resetView: fit, snapshot() {
      const v = view.current
      if (!v) return
      v.controls.update()
      v.renderer.render(v.scene, v.camera)
      return v.renderer.domElement.toDataURL('image/png')
    }, focusPoint(point) {
      const v = view.current
      if (!v) return
      const target = new THREE.Vector3(...lingtuToThree([point.x, point.y, point.z]))
      v.camera.position.add(target.clone().sub(v.controls.target))
      v.controls.target.copy(target); v.controls.update(); v.render()
    }, mapEditView(nextView) {
      const v = view.current
      const bounds = editRef.current?.selection ?? (cloud && mapEditCloudBounds(cloud.positions))
      if (!v || !bounds) return
      v.controls.enableDamping = false
      v.controls.update()
      frameMapEditView(v.camera, v.controls.target, bounds, nextView)
      v.controls.update()
      v.controls.enableDamping = !editRef.current?.selecting && !matchMedia('(prefers-reduced-motion: reduce)').matches
      v.render()
    } })
    return () => onReady?.(null)
  }, [cloud, onReady, fit])

  useEffect(() => {
    const v = view.current
    if (!v) return
    v.controls.enableRotate = !mapEdit?.selecting
    v.controls.enablePan = !mapEdit?.selecting
    v.controls.enableDamping = !mapEdit?.selecting && !matchMedia('(prefers-reduced-motion: reduce)').matches
    if (!mapEdit?.selecting) {
      pointer.current = null
      if (selectionRectangle.current) selectionRectangle.current.hidden = true
      v.controls.enabled = true
    }
    v.render()
  }, [mapEdit?.selecting])

  useEffect(() => {
    const v = view.current
    if (!v) return
    let geometry: THREE.Object3D | null
    if (displayCloud && !octomap) {
      geometry = createPcdLayer(displayCloud)
    } else geometry = octomap
      ? createOctomapLayer(octomap, 0, 0, 'height', voxelScale, Infinity)
      : createSavedMapLayer(points, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
    if (!geometry) { v.render(); return }
    const object = geometry as MapLayer
    layer.current = object
    v.scene.add(object)
    v.bounds.setFromObject(object)
    if (!v.fitted) { fit(); v.fitted = true }
    v.render()
    return () => {
      v.scene.remove(object)
      object.geometry.dispose()
      object.material.dispose()
      if (object instanceof THREE.InstancedMesh) object.dispose()
      layer.current = null
    }
  }, [points, displayCloud, octomap, voxelScale, fit])

  useEffect(() => {
    const object = layer.current
    if (!object) return
    const material = object.material
    if (material instanceof THREE.PointsMaterial) { material.size = pointSize; material.sizeAttenuation = false }
    if (material instanceof THREE.ShaderMaterial) {
      material.uniforms.pointSize.value = pointSize * view.current!.renderer.getPixelRatio()
      material.uniforms.opacity.value = opacity
    }
    material.opacity = opacity
    material.transparent = opacity < 1
    material.depthWrite = opacity === 1
    material.clippingPlanes = ceiling == null ? [] : [new THREE.Plane(new THREE.Vector3(0, -1, 0), ceiling)]
    material.needsUpdate = true
    view.current?.render()
  }, [points, displayCloud, octomap, voxelScale, pointSize, opacity, ceiling])

  useEffect(() => {
    const v = view.current
    if (!v || !grid || v.bounds.isEmpty()) return
    const size = v.bounds.getSize(new THREE.Vector3())
    const divisions = Math.max(2, Math.ceil(Math.max(size.x, size.z) / gridStep) + 2)
    const guide = new THREE.GridHelper(divisions * gridStep, divisions, 0x9ca3af, 0x9ca3af)
    guide.position.copy(v.bounds.getCenter(new THREE.Vector3()))
    guide.position.y = v.bounds.min.y - .01
    guide.material.transparent = true
    guide.material.opacity = .3
    v.scene.add(guide)
    v.render()
    return () => { v.scene.remove(guide); guide.dispose(); if (view.current === v) v.render() }
  }, [points, displayCloud, octomap, voxelScale, grid, gridStep])

  useEffect(() => {
    const v = view.current
    if (!v || !mapEdit || !cloud || !displayCloud) return
    const group = new THREE.Group()
    if (mapEdit.surface) {
      const geometry = new THREE.BufferGeometry()
      geometry.setAttribute('position', new THREE.Float32BufferAttribute(planePoints(mapEdit.surface), 3))
      const patch = new THREE.Points(geometry, new THREE.PointsMaterial({ color: 0xd49239, size: 3, sizeAttenuation: false }))
      patch.matrix.makeBasis(new THREE.Vector3(...lingtuToThree([1, 0, 0])),
        new THREE.Vector3(...lingtuToThree([0, 1, 0])), new THREE.Vector3(...lingtuToThree([0, 0, 1])))
      patch.matrixAutoUpdate = false
      group.add(patch)
    }
    const selection = mapEdit.selection
    if (selection) group.add(createMapEditBox(selection))
    const selected: number[] = []
    const removed: number[] = []
    if (selection) for (let i = 0; i < displayCloud.positions.length; i += 3) {
      const x = displayCloud.positions[i], y = displayCloud.positions[i + 1], z = displayCloud.positions[i + 2]
      if (pointInMapEditBounds(x, y, z, selection)) selected.push(x, y, z)
    }
    if (mapEdit.view === 'draft' && mapEdit.showDeleted && mapEdit.operations.length) {
      if (mapEdit.operations.every(isLocalMapEdit)) {
        for (let i = 0; i < cloud.positions.length; i += 3) {
          const x = cloud.positions[i], y = cloud.positions[i + 1], z = cloud.positions[i + 2]
          if (!mapEditKeepsPoint(x, y, z, mapEdit.operations)) removed.push(x, y, z)
        }
      } else {
        for (const value of mapEdit.preview?.preview_removed ?? []) removed.push(value)
      }
    }
    for (const [positions, color, alpha] of [[selected, 0xe6b96d, 1], [removed, 0xd79883, .25]] as const) {
      const highlight = createSavedMapLayer(positions, SAVED_MAP_Z_FLOOR, SAVED_MAP_Z_CEIL)
      if (!highlight) continue
      const material = highlight.material as THREE.PointsMaterial
      material.vertexColors = false
      material.color.setHex(color)
      material.size = pointSize + 1
      material.sizeAttenuation = false
      material.depthTest = false
      material.transparent = true
      material.opacity = alpha
      material.clippingPlanes = ceiling == null ? [] : [new THREE.Plane(new THREE.Vector3(0, -1, 0), ceiling)]
      highlight.renderOrder = 54
      group.add(highlight)
    }
    v.scene.add(group)
    v.render()
    return () => {
      v.scene.remove(group)
      group.traverse(object => {
        if (object instanceof THREE.Points || object instanceof THREE.LineSegments || object instanceof THREE.Mesh) {
          object.geometry.dispose()
          const materials = Array.isArray(object.material) ? object.material : [object.material]
          materials.forEach(material => material.dispose())
        }
      })
      if (view.current === v) v.render()
    }
  }, [mapEdit, cloud, displayCloud, pointSize, ceiling])

  useEffect(() => {
    const v = view.current
    if (!v || !noGoRegions?.length) return
    const regions = createNoGoLayer(noGoRegions)
    v.scene.add(regions); v.render()
    return () => {
      v.scene.remove(regions)
      regions.traverse(object => {
        if (object instanceof THREE.Mesh || object instanceof THREE.LineSegments) {
          object.geometry.dispose()
          const materials = Array.isArray(object.material) ? object.material : [object.material]
          materials.forEach(material => material.dispose())
        }
      })
      if (view.current === v) v.render()
    }
  }, [noGoRegions])

  useEffect(() => {
    const v = view.current
    if (!v || !inspectionPoints?.length) return
    const group = new THREE.Group()
    for (const point of inspectionPoints) {
      const marker = new THREE.Mesh(new THREE.SphereGeometry(.1, 10, 8),
        new THREE.MeshBasicMaterial({ color: 0x357eac, depthTest: false }))
      marker.position.set(...lingtuToThree([point.x, point.y, point.z])); marker.renderOrder = 60
      const canvas = document.createElement('canvas')
      canvas.width = 384; canvas.height = 56
      const context = canvas.getContext('2d')!
      context.font = '24px sans-serif'; context.textAlign = 'center'; context.fillStyle = '#233d52'
      context.fillText(point.name, 192, 36, 374)
      const label = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }))
      label.position.copy(marker.position).add(new THREE.Vector3(0, .35, 0)); label.scale.set(1.8, .263, 1); label.renderOrder = 61
      group.add(marker, label)
    }
    v.scene.add(group); v.render()
    return () => {
      v.scene.remove(group)
      group.traverse(object => {
        if (object instanceof THREE.Mesh) {
          object.geometry.dispose()
          const materials = Array.isArray(object.material) ? object.material : [object.material]
          materials.forEach(material => material.dispose())
        }
        if (object instanceof THREE.Sprite) { object.material.map?.dispose(); object.material.dispose() }
      })
      if (view.current === v) v.render()
    }
  }, [inspectionPoints])

  const updateRectangle = (event: PointerEvent<HTMLDivElement>) => {
    const active = pointer.current
    if (!active || active.id !== event.pointerId) return
    const rect = event.currentTarget.getBoundingClientRect()
    active.rectangle.x1 = event.clientX - rect.left
    active.rectangle.y1 = event.clientY - rect.top
    const box = selectionRectangle.current!
    box.style.left = `${Math.min(active.rectangle.x0, active.rectangle.x1)}px`
    box.style.top = `${Math.min(active.rectangle.y0, active.rectangle.y1)}px`
    box.style.width = `${Math.abs(active.rectangle.x1 - active.rectangle.x0)}px`
    box.style.height = `${Math.abs(active.rectangle.y1 - active.rectangle.y0)}px`
  }
  const clearPointer = (event: PointerEvent<HTMLDivElement>) => {
    pointer.current = null
    if (selectionRectangle.current) selectionRectangle.current.hidden = true
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId)
    if (view.current) view.current.controls.enabled = true
  }

  return <div ref={mount} className={styles.canvas} style={{ cursor: mapEdit?.selecting ? 'crosshair' : undefined,
    touchAction: mapEdit?.selecting ? 'none' : undefined }}
    onPointerDownCapture={event => {
      if (pointPicking && event.button === 0) pointPointer.current = { x: event.clientX, y: event.clientY }
      if (!mapEdit?.selecting || !displayCloud || event.button !== 0 || pointer.current) return
      event.preventDefault(); event.stopPropagation()
      const rect = event.currentTarget.getBoundingClientRect()
      const x = event.clientX - rect.left, y = event.clientY - rect.top
      pointer.current = { id: event.pointerId, rectangle: { x0: x, y0: y, x1: x, y1: y } }
      event.currentTarget.setPointerCapture(event.pointerId)
      if (view.current) view.current.controls.enabled = false
      selectionRectangle.current!.hidden = false
      updateRectangle(event)
    }}
    onPointerMoveCapture={updateRectangle}
    onPointerUpCapture={event => {
      const clicked = pointPointer.current
      pointPointer.current = null
      if (pointPicking && clicked && displayCloud && opacity > 0
        && Math.hypot(event.clientX - clicked.x, event.clientY - clicked.y) < 4) {
        const v = view.current!
        const rect = event.currentTarget.getBoundingClientRect()
        v.camera.updateMatrixWorld()
        const matrix = new THREE.Matrix4().multiplyMatrices(v.camera.projectionMatrix, v.camera.matrixWorldInverse)
        const point = pickVisiblePcdPoint(displayCloud.positions, new Float32Array(matrix.elements),
          rect.width, rect.height, event.clientX - rect.left, event.clientY - rect.top, 8, pointSize, ceiling ?? Infinity)
        if (point) onPointPick?.(point)
      }
      const active = pointer.current
      if (!active || active.id !== event.pointerId) return
      event.preventDefault(); event.stopPropagation()
      updateRectangle(event)
      const v = view.current, object = layer.current
      clearPointer(event)
      if (!v || !(object instanceof THREE.Points) || !displayCloud
        || Math.abs(active.rectangle.x1 - active.rectangle.x0) < 5 || Math.abs(active.rectangle.y1 - active.rectangle.y0) < 5) return
      const bounds = selectSavedMapBounds(displayCloud.positions, object, v.camera,
        event.currentTarget.getBoundingClientRect(), active.rectangle)
      if (bounds) onMapEditBounds?.(bounds)
    }}
    onPointerCancel={clearPointer}
    onKeyDown={event => {
    if (event.key === '+' || event.key === '=') { event.preventDefault(); zoom(.8) }
    if (event.key === '-') { event.preventDefault(); zoom(1.25) }
    if (event.key === 'Home') { event.preventDefault(); fit() }
  }}><div ref={selectionRectangle} hidden aria-hidden="true" style={{ position: 'absolute', pointerEvents: 'none',
    zIndex: 2, border: '1px solid #cf9a43', background: 'rgb(230 185 109 / 12%)' }} />
    <canvas ref={axes} className={styles.axes} role="img" aria-label="地图 XYZ 方向：红 X，绿 Y，蓝 Z" />
    {error && <p role="alert" className={styles.placeholder}>{error}</p>}</div>
}
