# Design System — LingTu Robot Control Dashboard

> Based on Linear's dark-mode-first design language, adapted for field inspection operations.
> AI agents: read PRODUCT.md first, then use this file for LingTu interaction and visual decisions.

## 0. Product and Operational Context

LingTu Web is the mission HMI for an inspection-first quadruped system, not a generic robot cockpit or a DDS/ROS topic browser. Its primary user is a trained field operator supervising one robot in substations, distribution rooms, cable tunnels, utility corridors, and similar power or energy facilities. Supervisors and equipment specialists review completion, evidence, and anomalies; integrators use separate advanced diagnostics.

The product promise is a repeatable inspection loop:

1. Plan a route and fixed inspection points.
2. Reach each point at a reproducible pose.
3. Capture evidence with visible quality and freshness.
4. Flag anomalies without hiding uncertainty.
5. Support human review, repair, and reinspection.

Every primary operational surface must answer four questions within a glance:

- What is the robot doing now?
- Is the robot and its environment safe?
- Who or what currently owns motion control?
- What does the operator need to do next?

Connectivity, safety, control ownership, and mission or motion state are independent dimensions. Do not collapse them into one “online” or “running” badge. An accepted command is not the same as a physically confirmed outcome; pending, accepted, rejected, latched, and recovered states need distinct language.

The default information hierarchy is task → route → point → evidence → review or reinspection. Transport names, topics, process health, and tuning belong in advanced diagnostics. Continuous manual driving should prefer a physical controller; the Web surface is for preflight, supervision, deliberate takeover, release, and recovery.

Current capability must remain visually honest. The verified inspection baseline is route, point, action, timestamp, RGB, pose, and detections. Thermal, gas, acoustic or partial-discharge sensing, trend analysis, automatic reports, and work-order integration are target capabilities until their runtime contracts and field evidence exist.

Design for strong sunlight, low light, gloves, weak or offline connectivity, dust, rain, cold, GPS denial, sensor drift, and robot operation outside direct sight. Critical field controls need large targets, explicit labels, non-color state cues, timestamps, and actionable recovery guidance.

## 1. Visual Theme & Atmosphere

LingTu's dashboard is a field-inspection mission interface for supervising a quadruped robot and its evidence loop. The canvas is deep navy (`#0a0e17`) — not pure black, but a blue-shifted darkness that remains legible in operational settings. Content emerges through precisely calibrated luminance steps: surfaces at `#0f1520`, borders at `#1e2a3a`, and text at `#e2e8f0`. Every element must support task comprehension, safety, control ownership, or evidence review.

The signature color is **Navigational Green** (`#00ff88`) — a high-energy accent used for "alive" states, active indicators, and primary actions. It carries a soft glow (`0 0 8px rgba(0,255,136,0.4)`) on active elements, creating a bioluminescent quality. Red (`#ef4444`) is reserved exclusively for stop/emergency/error states. Amber (`#f59e0b`) signals caution or transitional states (SLAM mapping mode, degraded conditions).

Typography uses a dual-family system: **Noto Sans SC** for all UI text (optimized for Chinese), **JetBrains Mono** for all data readouts, coordinates, metrics, and code. The monospace data layer is critical — this is an instrument panel where numbers must be instantly readable at a glance.

**Key Characteristics:**
- Dark-mode-native: `#0a0e17` page, `#0f1520` surfaces, `#151c2b` elevated, `#1a2235` active
- Single chromatic accent: Navigational Green `#00ff88` with glow shadow
- Semantic red (`#ef4444`) only for stop/error, amber (`#f59e0b`) for caution
- Dual typography: Noto Sans SC (UI) + JetBrains Mono (data)
- Semi-transparent borders (`rgba(255,255,255,0.06)` to `rgba(255,255,255,0.10)`)
- Task-dense field layout that prioritizes mission state, safety, ownership, and evidence
- Pulsing green dot (1.5s ease-in-out) for live/active indicators
- Step-blink animation (0.6s) for E-STOP/emergency states

## 2. Color Palette & Roles

### Background Surfaces
| Token | Value | Use |
|-------|-------|-----|
| `--bg` | `#0a0e17` | Page background, deepest canvas |
| `--surface` | `#0f1520` | Panels, sidebar, topbar |
| `--bg3` | `#151c2b` | Elevated surfaces, cards, inputs |
| `--bg4` | `#1a2235` | Active/hover states on surfaces |

### Text
| Token | Value | Use |
|-------|-------|-----|
| `--text` | `#e2e8f0` | Primary text — near-white, prevents eye strain |
| `--text-dim` | `#8892a4` | Secondary text, labels, metadata |
| `--text-faint` | `#5a6478` | Tertiary text, placeholders, disabled |

### Accent & Semantic
| Token | Value | Use |
|-------|-------|-----|
| `--accent` | `#00ff88` | Primary accent — online, active, success, CTA |
| `--accent-dim` | `#00cc6a` | Accent hover/pressed state |
| `--red` | `#ef4444` | Emergency stop, errors, danger |
| `--amber` | `#f59e0b` | Caution, SLAM mapping, degradation warning |
| `--blue` | `#3b82f6` | Informational, user chat bubbles |

### Border & Divider
| Token | Value | Use |
|-------|-------|-----|
| `--border` | `#1e2a3a` | Standard solid border |
| `--border-hi` | `#2a3548` | Emphasized border (active cards) |
| Subtle | `rgba(255,255,255,0.06)` | Ultra-thin dividers |
| Standard | `rgba(255,255,255,0.10)` | Card and input borders |

### Robotics-Specific Semantic Colors
| State | Color | Example |
|-------|-------|---------|
| 在线 / Online | `#00ff88` | Connection badge, heartbeat dot |
| 离线 / Offline | `#ef4444` | Disconnection badge |
| 空闲 / IDLE | `#00ff88` | Navigation state |
| 执行中 / EXECUTING | `#3b82f6` | Active navigation |
| 规划中 / PLANNING | `#f59e0b` | Path planning |
| 已到达 / ARRIVED | `#00ff88` | Goal reached |
| 失败 / FAILED | `#ef4444` | Navigation failure |
| 急停 / E-STOP | `#ef4444` + blink | Emergency stop active |
| SLAM 建图 | `#f59e0b` | Mapping mode |
| SLAM 导航 | `#00ff88` | Localization mode |

## 3. Typography Rules

### Font Families
- **UI**: `'Noto Sans SC', 'PingFang SC', 'Microsoft YaHei', system-ui, -apple-system, sans-serif`
- **Data/Mono**: `'JetBrains Mono', 'Fira Code', 'Cascadia Code', ui-monospace, monospace`

### Hierarchy

| Role | Font | Size | Weight | Line Height | Use |
|------|------|------|--------|-------------|-----|
| Logo | UI | 14px | 600 | 1.2 | Topbar brand name |
| Tab Label | UI | 13px | 500 | 1.2 | Tab bar buttons |
| Section Title | UI | 16px | 600 | 1.3 | Panel headers (地图管理, SLAM 模式) |
| Body | UI | 12px | 400 | 1.5 | General UI text |
| Body Emphasis | UI | 12px | 500 | 1.5 | Labels, active items |
| Small | UI | 11px | 400 | 1.4 | Hints, secondary info |
| Tiny | UI | 10px | 500 | 1.3 | Badges, tags, uppercase labels |
| Data Value | Mono | 11-14px | 500 | 1.2 | Coordinates, metrics, speeds |
| Data Large | Mono | 18px | 600 | 1.2 | SLAM stats, prominent metrics |
| Data Label | Mono | 10px | 400 | 1.3 | Stat labels (uppercase) |
| Chat Message | UI | 13px | 400 | 1.5 | Chat panel messages |
| Chat Input | UI | 13px | 400 | 1.4 | Input placeholder and text |
| Status Bar | Mono | 11px | 400 | 1.0 | Bottom bar values |

### Principles
- **Chinese-first and bilingual**: Operational labels default to clear Chinese and include complete English equivalents. Noto Sans SC handles CJK characters cleanly.
- **Mono for data**: Anything that is a number, coordinate, measurement, or machine value uses JetBrains Mono. This creates an instant visual distinction between "UI chrome" and "instrument data."
- **No bold above 600**: Maximum weight is 600 (semibold) for section titles. The system avoids typographic drama.
- **Field-legible density**: Desktop supervision can remain compact, but critical field actions use at least 44px targets, explicit labels, and type that remains legible in strong light.

## 4. Component Stylings

### Buttons

**Primary (Green)**
- Background: `#00ff88`
- Text: `#0a0e17` (dark on bright)
- Padding: 6px 12px
- Radius: 6px
- Hover: opacity 0.9
- Use: 保存地图, 激活, primary CTA

**Ghost**
- Background: `transparent`
- Text: `--text-dim`
- Border: `1px solid --border`
- Radius: 6px
- Hover: border-color → `--text-dim`, text → `--text`
- Use: 刷新, secondary actions

**Danger (Red)**
- Background: `rgba(239, 68, 68, 0.12)`
- Text: `#ef4444`
- Border: `1px solid #ef4444`
- Radius: 6px
- Hover: background → `#ef4444`, text → white, glow shadow
- Use: 紧急停止, 删除

**Tiny (Inline)**
- Background: transparent
- Text: `--text-dim`
- Border: `1px solid --border`
- Radius: 4px
- Padding: 4px 8px
- Font: 10px
- Use: Card actions (激活, 重命名, 3D 预览)

**Icon Button**
- Background: transparent
- Color: `--text-dim`
- Border: none
- Radius: 4px
- Hover: color → `--text`
- Use: Settings gear, close buttons

### Cards

**Map Card**
- Background: `--surface`
- Border: `1px solid --border`
- Radius: 6px
- Padding: 10px 14px
- Hover: border-color → `--text-dim`
- Active variant: border-color → `--accent`, background += `rgba(0,255,136,0.04)`

### Tags / Badges

**Status Badge**
- Padding: 2px 8px
- Radius: 10px
- Font: 10px weight 500
- Online: bg `rgba(0,255,136,0.12)`, text `#00ff88`
- Offline: bg `rgba(239,68,68,0.12)`, text `#ef4444`

**Data Tag**
- Padding: 1px 6px
- Radius: 3px
- Font: 10px weight 500
- PCD: bg `rgba(59,130,246,0.15)`, text `#60a5fa`
- Tomogram: bg `rgba(0,255,136,0.10)`, text `#00ff88`
- Empty: bg `rgba(255,255,255,0.05)`, text `--text-dim`

### Inputs

**Chat Input**
- Background: `--bg3`
- Text: `--text`
- Border: `1px solid --border`
- Radius: 6px
- Padding: 8px 12px
- Focus: border-color → `--accent`
- Placeholder: `--text-faint`

### Toast Notifications
- Position: fixed bottom-right, above status bar
- Radius: 6px
- Font: 11.5px mono
- Auto-dismiss: 2.5s
- Success: bg `rgba(0,255,136,0.1)`, border accent-dim, text `#00ff88`
- Error: bg `rgba(239,68,68,0.12)`, border red-dim, text `#ef4444`
- Info: bg `--bg3`, border `--border-hi`, text `--text-dim`
- Animation: slide-up 0.18s ease-out

## 5. Layout Principles

### Structure
```
┌─ Topbar (44px) ──────────────────────────────────────┐
│ Logo + Badge | Stats (位置/导航/SLAM) | Settings      │
├─ Tab Bar (32px) ─────────────────────────────────────┤
│ 控制台 | 地图管理 | SLAM 模式                          │
├──────────────────────────────────────────────────────┤
│                                                       │
│           Main Content (flex: 1)                      │
│   Console: Camera (flex-1) + Chat (320px fixed)       │
│   Map: Centered list (max-width 900px)                │
│   SLAM: Centered panel (max-width 680px)              │
│                                                       │
├─ Status Bar (28px) ──────────────────────────────────┤
│ 位置 | 航向 | 速度 | 导航 | SLAM Hz | 运行时长 | 版本  │
└──────────────────────────────────────────────────────┘
```

### Spacing
- Base unit: 4px
- Scale: 4, 6, 8, 10, 12, 14, 16, 20, 24, 32px
- Topbar padding: 0 14px
- Panel padding: 20px
- Card padding: 10px 14px
- Compact gaps: 6-8px between items

### Responsive (single breakpoint)
- `>900px`: Camera + Chat side-by-side (grid: 1fr 320px)
- `≤900px`: Camera stacks above Chat (grid: 1fr, chat height 240px)

### Border Radius Scale
| Size | Value | Use |
|------|-------|-----|
| Micro | 3px | Tags, tiny badges |
| Standard | 4px | Icon buttons, tiny buttons |
| Comfortable | 6px | Buttons, cards, inputs |
| Badge | 10px | Status badges |
| Full Pill | 9999px | Round indicators |

## 6. Depth & Elevation

Depth on dark surfaces is communicated through background luminance, not shadows.

| Level | Background | Border | Use |
|-------|-----------|--------|-----|
| Canvas | `#0a0e17` | none | Page background |
| Surface | `#0f1520` | `--border` | Topbar, sidebar, panels |
| Elevated | `#151c2b` | `--border` | Cards, inputs, dropdowns |
| Active | `#1a2235` | `--border-hi` | Hover states, active cards |
| Accent | `rgba(0,255,136,0.04-0.08)` | `--accent` | Active map card, selected state |

No drop shadows. No gradients. Depth = luminance step.

## 7. Motion & Animation

| Effect | Duration | Easing | Use |
|--------|----------|--------|-----|
| Hover transition | 0.15s | ease | Border/color/background changes |
| Pulse (live dot) | 1.5s | ease-in-out | Camera LIVE badge, heartbeat |
| Blink (E-STOP) | 0.6s | steps | Emergency stop text |
| Spinner | 0.7s | linear | SLAM mode switching pending |
| Toast slide-in | 0.18s | ease-out | Notification appearance |

No layout animations. No page transitions. Operational HMIs should not distract.

## 8. Do's and Don'ts

### Do
- Use `#0a0e17` as the base canvas — never pure black `#000000`
- Use `#e2e8f0` for primary text — never pure white `#ffffff`
- Use `#00ff88` exclusively for positive/active states — never decoratively
- Use JetBrains Mono for ALL numbers, coordinates, and metrics
- Use Noto Sans SC for ALL UI labels and text
- Route every user-facing string through the locale helper with clear Chinese and complete English
- Keep buttons nearly transparent (ghost style) except for primary CTAs
- Use luminance stepping for elevation (darker = deeper, lighter = elevated)
- Use semi-transparent borders on dark surfaces
- Reserve red exclusively for stop/error/danger

### Don't
- Don't use warm colors in the UI chrome — the palette is cool navy + green
- Don't use shadows for depth — use background luminance steps
- Don't use font weight above 600 — no bold in this system
- Don't add decorative elements, gradients, or patterns
- Don't ship a user-facing label in only one language when the surface supports locale switching
- Don't make the camera feed smaller than the chat panel
- Don't animate layout changes — instant tab switches, no slide transitions
- Don't use more than 3 colors in any single view (bg + accent + one semantic)

## 9. Agent Prompt Guide

### Quick Token Reference
```
Page bg:      #0a0e17
Surface:      #0f1520
Elevated:     #151c2b
Border:       #1e2a3a
Text:         #e2e8f0
Text dim:     #8892a4
Text faint:   #5a6478
Accent green: #00ff88
Red:          #ef4444
Amber:        #f59e0b
Blue:         #3b82f6
Font UI:      'Noto Sans SC', system-ui
Font Mono:    'JetBrains Mono', monospace
Base size:    12px
Radius:       6px
```

### Example Prompts
- "Create a robot status card on `#0f1520` background. Title at 12px Noto Sans SC weight 500, color `#8892a4`, uppercase. Value at 18px JetBrains Mono weight 600, color `#00ff88`. Border `1px solid #1e2a3a`, radius 6px, padding 12px."
- "Build a navigation tab bar: height 32px, bg `#0f1520`, border-bottom `1px solid #1e2a3a`. Tabs at 13px Noto Sans SC weight 500. Active: color `#00ff88` + 2px bottom border `#00ff88`. Inactive: color `#5a6478`."
- "Design a chat bubble: user (right-aligned, bg `rgba(59,130,246,0.15)`, border-radius 12px 12px 2px 12px). System (left-aligned, bg `#151c2b`, border-radius 12px 12px 12px 2px). Text 13px, mono for coordinates."
- "Create an emergency stop button: bg `rgba(239,68,68,0.12)`, border `1px solid #ef4444`, text `#fff` JetBrains Mono 12px weight 700, radius 6px. Hover: bg solid `#ef4444`, glow `0 0 14px rgba(239,68,68,0.3)`."
