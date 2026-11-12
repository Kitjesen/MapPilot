/**
 * Mount a live avatar. This is the client's own component, `p_`: it builds the
 * driver and clip machine, caps frames at maxFrameRate, skips work while the tab
 * is hidden or the element is off screen, follows the pointer when asked, and
 * diffs attributes instead of rebuilding the SVG. Nothing here adds behaviour.
 */
import { client } from './engine.mjs';
import { appearanceOf } from './render.mjs';

/**
 * @param {Element} element the node the SVG is drawn into and sized from
 * @param {object} options the client's own prop names, plus `shape`/`color`
 *   sugar; omit `size` to let the element's box drive it, as the client does
 * @returns {{
 *   update(props: object): void, setState(state: string): void, setBody(body: string): void,
 *   setPlaying(playing: boolean): void, setExpression(name: string, duration?: number): void,
 *   blink(): void, look(angle: number): void, restart(): void, poke(name: string, strength?: number): void,
 *   gaze(point: {x: number, y: number}|null): void, destroy(): void
 * }}
 */
export function createAvatar(element, options = {}) {
  const controller = client.p_(appearanceOf(options));
  const detach = controller.attach({ host: element, canvas: element });
  return { ...controller, destroy: detach };
}
