/**
 * One call, one SVG string, drawn by the client renderer.
 *
 * The lifecycle below mirrors the client's own avatar component (`p_`): normalise
 * through `f_`, build the driver with `eo`, wrap it in the clip machine `go`,
 * settle or step it, then hand the frame to `Xg` and serialise with `hp`. Where a
 * number is needed it is read from the client rather than written here.
 */
import {
  client, bodyFor, colorValue, pinnedColor, resolveAvatar,
} from './engine.mjs';

const {
  c_, d_, eo, f_, go, hp, l_, no, o_, s_, u_, Xg,
} = client;

/**
 * @param {object} options
 * @param {string} [options.shape] picker name, e.g. 'pumpkin'
 * @param {string} [options.agentId] hashed for a shape and colour when none is given
 * @param {string|null} [options.avatarShape] client's own override field
 * @param {string|null} [options.avatarColor] client's own override field
 * @param {string} [options.color] colour id or hex
 * @param {string} [options.state] one of STATES
 * @param {number} [options.size] CSS pixels
 * @param {number|string} [options.seed]
 * @param {number|null} [options.time] omit for the client's still frame, or give
 *   seconds to step the driver to that moment the way the play loop would
 * @param {string} [options.id] id prefix; one per element on the page
 * @param {number} [options.fill] sprite coverage, the client default is .6
 * @param {number} [options.resolution] overrides the fill-derived sprite size
 * @param {boolean} [options.isProof] the client's shaded-mesh debug view
 * @returns {string} an `<svg>` document
 */
export function renderAvatarSvg(options = {}) {
  const shape = shapeOf(options);
  const props = f_({ size: 96, ...appearanceOf(options, shape) });
  const reduced = d_();
  const isStatic = props.isStatic || reduced;
  const playing = props.isPlaying && !isStatic;
  const size = typeof props.size === 'number' ? props.size : 0;
  const driver = eo({ body: props.body });
  const machine = props.state === void 0
    ? null
    : go(driver, props.state, {
      seed: u_(String(options.seed ?? props.id)),
      isCompact: size === 0 || size < s_,
      isResting: !playing,
    });
  const resolution = props.resolution ?? Math.round(size * no(props.fill) * 100) / 100;

  if (options.time == null) {
    // The component's own static procedure: pause, rewind, run the clip once.
    driver.pause();
    driver.seek(0);
    for (let index = 0; index < c_; index++) driver.advance(l_);
  } else {
    const step = 1 / Math.max(1, props.maxFrameRate);
    for (let elapsed = 0; elapsed < options.time; elapsed += step) {
      if (playing) machine?.advance(step);
      driver.advance(step);
    }
  }

  if (options.expression != null) {
    driver.holdFace(options.expression);
    for (let index = 0; index < c_; index++) driver.advance(l_);
  }

  return hp(Xg(driver.frame(), props.appearance, resolution, props.id, {
    isProof: props.isProof,
    displaySize: size > 0 ? size : resolution,
    extent: '100%',
  }));
}

/** The picker name wins, then an explicit avatarShape, then the agent id recipe. */
function shapeOf(options) {
  if (options.shape) return options.shape;
  if (options.avatarShape) return options.avatarShape;
  if (options.agentId) return resolveAvatar(options).shape;
  throw TypeError('renderAvatarSvg needs shape, avatarShape or agentId');
}

function colorOf(options, shape) {
  const pinned = pinnedColor(shape);
  const requested = pinned ?? options.color ?? options.avatarColor
    ?? (options.agentId ? resolveAvatar(options).color : null);
  if (requested == null) throw TypeError('renderAvatarSvg needs color, avatarColor or agentId');
  return colorValue(requested) ?? requested;
}

/**
 * Picker options to the prop names the client component takes. Exported because
 * createAvatar needs the same mapping without the still-render size default.
 */
export function appearanceOf(options, shape = shapeOf(options)) {
  const { color, ...rest } = options;
  return {
    ...rest,
    id: options.id ?? `avatar-${shape}`,
    body: bodyFor(shape),
    appearance: options.appearance ?? o_(colorOf(options, shape)),
    state: options.state ?? 'idle',
  };
}
