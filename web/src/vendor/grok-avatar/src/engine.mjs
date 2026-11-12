/**
 * The single bridge to the client renderer.
 *
 * Everything visual in this package comes out of data/extracted/src/avatar-engine.js,
 * which is the app's own declarations lifted out of the bundle untouched. This file
 * only names things: it re-exports the client's declarations under readable labels
 * and reads its tables. It contains no geometry, no colour maths and no motion, and
 * nothing here should grow any.
 *
 * `client` is exported as the escape hatch for anything not surfaced below.
 */
import * as client from '../data/extracted/src/avatar-engine.js';

export { client };

/** Picker names, in the client's order. `blob`/`flower`/`heart`/`star` are aliases. */
export const SHAPES = client.ln;

/** Renderable bodies, i.e. what the renderer actually accepts. */
export const BODIES = client.ei;

/** The colour table: {id, label, value}. */
export const COLORS = client.nn;

/** Every state the clip machine knows, from the client's state table. */
export const STATES = Object.keys(client.lo);

/** Below this size the client swaps to the compact clips. */
export const COMPACT_SIZE = client.s_;

/** A colour id, or null when the colour is free to choose. */
export const colorValue = (id) => client.nn.find((entry) => entry.id === id)?.value ?? null;

/** The colour a shape forces, or null. In 0.78.1 only the pumpkin does. */
export const pinnedColor = (shape) =>
  client.cn.find((entry) => entry.shape === shape)?.pickerColor ?? null;

/** Picker name to render body: blob to sphere, flower to starFlower, heart to heartChubby, star to star6. */
export const bodyFor = (shape) => client.i_(shape);

/**
 * The paint record the emitter wants. One argument cuts the eyes out and paints
 * the body flat; a second stops cutting and paints the eyes instead.
 */
export const paintOf = (ink, spot) => client.o_(ink, spot);

/**
 * agent id to shape and colour, as the client derives them. `jn`, not `An`:
 * An only accepts the 25 picker shapes, so an explicit pumpkin never survives it.
 */
export const resolveAvatar = (input) => client.jn(
  typeof input === 'string' ? { agentId: input } : input,
);

/** The client's own prop normaliser; its defaults are the package defaults. */
export const defaults = client.f_;
