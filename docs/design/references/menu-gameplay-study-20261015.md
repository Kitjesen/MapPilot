# Generated menu layout study

- Mode: built-in image generation, using an existing gameplay frame as reference.
- Image: [menu-gameplay-study-20261015.png](menu-gameplay-study-20261015.png).
- Source: `build/ue-drive-range-20261014/manual-20261014-212737-635f42c4/video-midpoint.png`.
- Status: user accepted the left-aligned black/white visual direction, but rejected
  the six technical menu actions. The approved five player-facing actions are in
  [runtime-hud.md](../runtime-hud.md).
- This is generated design material, not a pixel-identical runtime screenshot or
  simulation evidence. It must not be baked into the game as a fake scene/menu.

## Generation prompt

```text
Use case: ui-mockup. Asset: original shippable-looking Chinese pause/menu screen for the LingTu robot driving simulation. Input image 1 is the exact gameplay-background edit target, NOT a UI style reference. Remove every existing HUD/text/keycap from that image, then redesign ONLY the screen-space menu. Preserve the factory roadway, cones, wheel-legged robot geometry, robot position, camera perspective, materials, and lighting exactly as supplied. Do not beautify or replace the environment, invent more detail, relocate the robot, or turn it into concept art. 16:9 landscape, one complete screen, no presentation board or device frame.
Art direction: a quietly confident, authored PC game menu, restrained neutral charcoal/white with a slight natural warmth, not futuristic robotics dashboard. Asymmetric layout, compact navigation on the left approximately x=110..500, with the center and right staying as the gameplay view. A soft neutral-black horizontal darkening gradient behind the left menu fades fully away near the middle; no blue tint, no heavy global darkening. No cards, no right-side summary panel, no glassmorphism, no rounded boxes, no cyan, no gold accents, no neon, no glows, no ornamental techno lines, no fake gauges, no logos, no filler microcopy. A substantial but not oversized plain title, crisp CJK sans serif, natural typographic rhythm and generous vertical spacing. Focused entry is white and slightly heavier with only a small white chevron at its left; unfocused entries light grey, no filled blue selection pill. One very subtle short divider before the final exit action. The only title text is exactly "菜单". Navigation text, each once, in this order: "继续操作", "切换视角", "运行信息", "场景预览", "开始记录", "结束会话". Selected item is "继续操作". Bottom-left a small plain "Esc  返回" and below or beside it a readable understated note exactly "菜单不会自动暂停仿真". No other text, no statistics or invented state, no English section headings, no top corner HUD. Render the requested Chinese labels legibly. The result must feel like a practical in-game menu designed by a game UI artist rather than an AI-generated web dashboard. All menu elements must be feasible in native UE Slate; the background must remain the supplied real gameplay scene.
```
