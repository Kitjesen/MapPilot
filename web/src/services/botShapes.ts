// The eight classic IDs and paths remain stable for saved profiles.
export const LINGTU_HEADS = {
  orb: { label: '圆润', path: 'M14 8A6 6 0 1 1 2 8A6 6 0 1 1 14 8Z' },
  capsule: { label: '宽屏', path: 'M5 3H11C13.5 3 15 5 15 8S13.5 13 11 13H5C2.5 13 1 11 1 8S2.5 3 5 3Z' },
  square: { label: '方头', path: 'M5 2H11Q14 2 14 5V11Q14 14 11 14H5Q2 14 2 11V5Q2 2 5 2Z' },
  bean: { label: '豆豆', path: 'M12.8 3.4C15.1 5.8 14.2 11.8 10.5 13.6C7.3 15.2 2 12.8 1.8 9.1C1.6 6.5 4.1 6 5.2 4.2C6.9 1.3 10.5 1 12.8 3.4Z' },
  drop: { label: '水滴', path: 'M8 1.4C9 1.4 9.4 3.5 11 4.9C13 6.5 14 8 14 10C14 13 11.4 14.6 8 14.6S2 13 2 10C2 8 3 6.5 5 4.9C6.6 3.5 7 1.4 8 1.4Z' },
  cloud: { label: '云朵', path: 'M4.1 12.7C1.4 12.7 .7 8.8 3.1 7.4C2.3 4.8 5.4 2.7 7.4 4C9.2 1.9 12.9 3.1 12.8 5.9C15.8 6.1 16 10.4 13.5 11.5C12.3 14.5 9.5 14.7 7.8 13.2C6.7 14.2 5 14.1 4.1 12.7Z' },
  bud: { label: '芽芽', path: 'M3.2 11.8C1.8 10 2.4 7.4 4.4 6.6C2.6 3.1 5.9 1 8 4.3C10.2 1 13.3 3.1 11.6 6.6C13.7 7.4 14.2 10 12.8 11.8C10.8 14.5 5.2 14.5 3.2 11.8Z' },
  pebble: { label: '软石', path: 'M3.7 3.2C6.2 1.7 11.7 2.2 13.3 4.7C14.5 6.7 14.9 10.6 12.7 12.9C10.4 15.1 5.4 14.5 3.4 12.5C1.4 10.5 1.3 5 3.7 3.2Z' },

  // LingTu redraws from the supplied eighteen-shape reference, not upstream IDs.
  'morph-stone': { label: '石', path: 'M7.3 1.2C10.2 1 11.2 2.5 12.2 4.3C13.2 5.9 14.7 6.6 14.8 9.4C15 12.9 11.9 14.7 8.5 14.8C4.1 15 1 11.8 1.1 8C1.2 4.8 3.9 1.4 7.3 1.2Z' },
  'morph-listen': { label: '听', path: 'M5.8 1.3C8.1 .7 9 2.3 8.4 4C7.7 6 8.6 7.4 10.6 7C12.5 6.5 14.5 6.5 14.9 9.1C15.5 12.5 12.1 14.9 8.7 14.9C4.3 14.9 1 11.7 1 7.7C1 4.6 2.8 1.9 5.8 1.3Z', eyeX: .4, eyeY: 2 },
  'morph-plus': { label: '加', path: 'M6.2 1.5H9.8Q11.3 1.5 11.5 3.1L11.8 4.5Q11.9 5 12.5 5.1L13.5 5.3Q15 5.6 15 7.4V9Q15 10.9 13.4 11.2L12 11.4Q11.5 11.5 11.4 12L11.1 13.3Q10.8 14.6 9.2 14.6H6.4Q4.8 14.6 4.5 13.1L4.2 11.8Q4.1 11.3 3.5 11.2L2.5 10.9Q1 10.5 1 8.8V7.5Q1 5.8 2.6 5.3L3.6 5Q4.1 4.9 4.2 4.3L4.5 3Q4.7 1.5 6.2 1.5Z' },
  'morph-carry': { label: '携', path: 'M8.2 1C11.6 .7 13.2 2.3 12.9 4.4C12.6 6.5 10.9 6.5 9.9 6.3C9.1 6.1 8.9 6.9 8.9 7.9C8.9 8.9 9.2 9.3 10.1 9C12.6 8.3 14.3 9.4 14.3 11.4C14.3 13.7 11.4 15 8.5 15C3.7 15 1.2 11.8 1.2 8C1.2 4.4 4 1.4 8.2 1Z', eyeX: -2.6 },
  'morph-point': { label: '点', path: 'M8 1C9.7 1 14.4 5.4 14.4 8C14.4 10.9 10 15 8 15C5.8 15 1.6 10.7 1.6 8C1.6 5.2 6.3 1 8 1Z' },
  'morph-warm': { label: '暖', path: 'M8 2C12.4 2 14 5.5 14.8 9.8C15.5 13.9 13.3 14 8.1 14C2.9 14 1 13.7 1.2 10.3C1.5 6 3.6 2 8 2Z' },
  'morph-sprout': { label: '芽', path: 'M8 1C10.2 1 13.9 6.1 13.9 9.7C13.9 13.3 11.7 15 8 15C4.3 15 2.1 12.9 2.1 9.7C2.1 6 5.7 1 8 1Z' },
  'morph-toward': { label: '向', path: 'M8 1.7C10 1.7 14.9 7.6 14.9 10.9C14.9 13.5 13.1 14.5 11.2 13.5L9 12.2Q8 11.6 7 12.2L4.8 13.5C2.8 14.5 1.1 13.5 1.1 10.9C1.1 7.6 6 1.7 8 1.7Z' },
  'morph-gate': { label: '门', path: 'M8 1.3C12.8 1.3 14.7 5.8 14.7 11.6C14.7 13.8 14.6 14.2 12.6 14.2H11.6C9.1 14.2 10.4 10.5 8 10.5C5.6 10.5 6.9 14.2 4.4 14.2H3.4C1.4 14.2 1.3 13.8 1.3 11.6C1.3 5.8 3.2 1.3 8 1.3Z', eyeY: -1 },
  'morph-flow': { label: '流', path: 'M8.8 1.6C12.3 1.5 13.8 2.7 13.2 4.7C12.8 6.2 11.3 6.1 10.4 6.1C9.1 6.1 8.9 6.7 8.9 7.7C8.9 8.8 9.3 9.3 10.6 9.1C13 8.7 14.9 9.5 14.9 11.4C14.9 13.4 12.1 14.6 8.8 14.6C4.4 14.6 1 11.8 1 8.2C1 4.3 4.8 1.8 8.8 1.6Z', eyeX: -2.6 },
  'morph-gather': { label: '聚', path: 'M8 1.2C10.2 1.2 11.3 2.6 11.8 4.8C12.5 7.6 14.8 8 14.8 10.6C14.8 13.4 12.6 14.9 10.2 13.8Q8 12.5 5.8 13.8C3.4 14.9 1.2 13.4 1.2 10.6C1.2 8 3.5 7.6 4.2 4.8C4.7 2.6 5.8 1.2 8 1.2Z' },
  'morph-cup': { label: '纳', path: 'M3 2.3C5 2.1 6 5.5 8 5.5C10 5.5 11 2.1 13 2.3C15.4 2.5 15.1 7.7 13.6 10.8C12.5 13 10.6 14.3 8 14.3C5.4 14.3 3.5 13 2.4 10.8C.9 7.7 .6 2.5 3 2.3Z', eyeY: 1 },
  'morph-watch': { label: '望', path: 'M8 1.1C11.1 1.1 12.9 3.3 12.5 6.2C12.2 8.3 13.7 9.3 13.7 11.3C13.7 14.1 11.5 14.9 8 14.9C4.5 14.9 2.3 14.1 2.3 11.3C2.3 9.3 3.8 8.3 3.5 6.2C3.1 3.3 4.9 1.1 8 1.1Z', eyeY: -1.2 },
  // The second contour cuts a narrow spiral gap; eyes stay on the inner body.
  'morph-loop': { label: '循', path: 'M6.5 1.1C11.4 .4 15 4 15 8.5C15 12.6 12.2 15 8 15C3.9 15 1.1 13.4 1.1 10.2C1.1 7.2 3.6 5.5 6.1 5.1C8.6 3.9 11.4 5.1 11.6 8C11.8 10.2 10.6 11.6 8.9 12.4L9.4 12.8C11.8 11.8 12.8 9.6 12.2 7.1C11.6 4.3 9.2 3 6.7 3.4L5 4.1L3.6 3.7L3.2 2.8L3.6 2C4.4 1.4 5.3 1.2 6.5 1.1ZM6.4 4.3C9.5 3.2 12.7 5.5 12.7 8.4C12.7 10.7 11.3 12.3 9.4 13L8.8 12.4C11 11.3 11.9 9.7 11.2 7.6C10.6 5.5 8.8 4.7 6.6 5.2L5 5L6.4 4.3Z', eyeX: -1.4, eyeY: 1.2 },
  'morph-rest': { label: '息', path: 'M5.8 3.1C8.5 2.8 9 4.8 11.8 5.4C14 5.8 15 6.5 14.8 8.8C14.5 12.6 12.1 12.2 8.9 12.2C5.7 12.2 2.2 12.5 1.3 9.4C.3 6.1 2.5 3.5 5.8 3.1Z', eyeX: -.8 },
  'morph-balance': { label: '衡', path: 'M4.3 3.2C5.9 3.2 6.7 4.3 8 4.3C9.3 4.3 10.1 3.2 11.7 3.2C14.1 3.2 15 5.6 15 8C15 10.4 14.1 12.8 11.7 12.8C10.1 12.8 9.3 11.7 8 11.7C6.7 11.7 5.9 12.8 4.3 12.8C1.9 12.8 1 10.4 1 8C1 5.6 1.9 3.2 4.3 3.2Z' },
  'morph-question': { label: '问', path: 'M7.6 1C11.7 .6 14.1 3 14 6C13.9 8.9 11.6 9.9 9.5 11.1L10.1 11.9C12.3 11.4 13.6 12 13.4 13.4C13.2 14.9 10.5 15.1 8.7 15C5.7 14.9 4.3 13.6 4.4 11.5C4.5 9.2 6.5 8.6 8.5 7.5L8 7C5.9 7.2 3.9 9.2 2.2 7.4C.2 5.3 2.4 1.5 7.6 1Z', eyeX: -.5, eyeY: -3 },
  'morph-stack': { label: '叠', path: 'M8 2.1C11 2.1 12.4 4.1 12.7 6.6C12.9 7.7 15 8.3 15 10.6C15 13.6 12.9 14 8 14C3.1 14 1 13.6 1 10.6C1 8.3 3.1 7.7 3.3 6.6C3.6 4.1 5 2.1 8 2.1Z' },
} as const

// Values are the official picker names, including its gated pumpkin shape.
export const GROK_SHAPES = {
  'grok-blob': { label: '圆球', shape: 'blob' },
  'grok-pebble': { label: '卵石', shape: 'pebble' },
  'grok-bean': { label: '豆形', shape: 'bean' },
  'grok-egg': { label: '蛋形', shape: 'egg' },
  'grok-squircle': { label: '圆角方', shape: 'squircle' },
  'grok-tablet': { label: '平板', shape: 'tablet' },
  'grok-capsule': { label: '胶囊', shape: 'capsule' },
  'grok-cylinder': { label: '圆柱', shape: 'cylinder' },
  'grok-hex': { label: '六边形', shape: 'hex' },
  'grok-gem': { label: '宝石', shape: 'gem' },
  'grok-crystal': { label: '晶簇', shape: 'crystal' },
  'grok-wedge': { label: '楔形', shape: 'wedge' },
  'grok-shield': { label: '盾形', shape: 'shield' },
  'grok-dome': { label: '穹顶', shape: 'dome' },
  'grok-arch': { label: '拱门', shape: 'arch' },
  'grok-cloud': { label: '云团', shape: 'cloud' },
  'grok-teardrop': { label: '泪滴', shape: 'teardrop' },
  'grok-leaf': { label: '叶片', shape: 'leaf' },
  'grok-square': { label: '方形', shape: 'square' },
  'grok-sparkle': { label: '闪光', shape: 'sparkle' },
  'grok-clover': { label: '三叶草', shape: 'clover' },
  'grok-flower': { label: '花朵', shape: 'flower' },
  'grok-heart': { label: '爱心', shape: 'heart' },
  'grok-house': { label: '小屋', shape: 'house' },
  'grok-star': { label: '六角星', shape: 'star' },
  'grok-pumpkin': { label: '南瓜', shape: 'pumpkin' },
} as const

export type BotShape = keyof typeof LINGTU_HEADS | keyof typeof GROK_SHAPES

export const BOT_SHAPES = Object.fromEntries([
  ...Object.entries(GROK_SHAPES),
  ...Object.entries(LINGTU_HEADS),
].map(([id, shape]) => [id, shape.label])) as Record<BotShape, string>

export const BOT_SHAPE_GROUPS: Array<{ label: string; shapes: BotShape[] }> = [
  { label: '立体形象', shapes: Object.keys(GROK_SHAPES) as BotShape[] },
  { label: 'LingTu 重绘', shapes: Object.keys(LINGTU_HEADS).filter(id => id.startsWith('morph-')) as BotShape[] },
  { label: '经典外形', shapes: Object.keys(LINGTU_HEADS).filter(id => !id.startsWith('morph-')) as BotShape[] },
]
