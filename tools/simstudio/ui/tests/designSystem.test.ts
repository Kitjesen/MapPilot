import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";

type Rgb = [number, number, number];

const readProjectFile = (relativePath: string): string =>
  readFileSync(new URL(`../${relativePath}`, import.meta.url), "utf8");

const parseHexColor = (hex: string): Rgb => {
  const channels = hex.slice(1).match(/../g);
  assert.ok(channels);
  assert.equal(channels.length, 3);
  return channels.map((channel) => Number.parseInt(channel, 16)) as Rgb;
};

const relativeLuminance = (rgb: Rgb): number => {
  const linear = rgb.map((channel) => {
    const value = channel / 255;
    return value <= 0.04045 ? value / 12.92 : ((value + 0.055) / 1.055) ** 2.4;
  });
  return 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2];
};

const contrastRatio = (foreground: Rgb, background: Rgb): number => {
  const foregroundLuminance = relativeLuminance(foreground);
  const backgroundLuminance = relativeLuminance(background);
  const lighter = Math.max(foregroundLuminance, backgroundLuminance);
  const darker = Math.min(foregroundLuminance, backgroundLuminance);
  return (lighter + 0.05) / (darker + 0.05);
};

const rootDeclarations = (css: string): Map<string, string> => {
  const root = css.match(/:root\s*\{([\s\S]*?)\n\}/);
  assert.ok(root);
  const entries = [...root[1].matchAll(/(--[\w-]+):\s*([^;]+);/g)].map(
    (match): [string, string] => [match[1], match[2].trim()],
  );
  return new Map(entries);
};

const designTokenDeclarations = (): Map<string, string> =>
  new Map([
    ...rootDeclarations(readProjectFile("src/design-system/primitives.css")),
    ...rootDeclarations(readProjectFile("src/design-system/semantic.css")),
    ...rootDeclarations(readProjectFile("src/design-system/components.css")),
  ]);

const resolveToken = (
  declarations: Map<string, string>,
  name: string,
  seen: Set<string> = new Set(),
): string => {
  assert.equal(seen.has(name), false, `token cycle at ${name}`);
  const value = declarations.get(name);
  assert.ok(value, `missing token ${name}`);
  const reference = value.match(/^var\((--[\w-]+)\)$/);
  if (!reference) return value;
  return resolveToken(declarations, reference[1], new Set([...seen, name]));
};

test("design-system layers load in primitive semantic component order", () => {
  const indexCss = readProjectFile("src/design-system/index.css");
  const imports = [...indexCss.matchAll(/@import\s+"\.\/(.+?)"/g)].map(
    (match) => match[1],
  );

  assert.deepEqual(imports, ["primitives.css", "semantic.css", "components.css"]);
});

test("semantic and component layers do not introduce raw colors", () => {
  const rawColor = /#[0-9a-f]{3,8}\b|\brgba?\(/i;
  const semanticCss = readProjectFile("src/design-system/semantic.css");
  const componentCss = readProjectFile("src/design-system/components.css");

  assert.equal(rawColor.test(semanticCss), false);
  assert.equal(rawColor.test(componentCss), false);
});

test("the public token contract includes lightfield HUD truth and motion roles", () => {
  const css = [
    readProjectFile("src/design-system/primitives.css"),
    readProjectFile("src/design-system/semantic.css"),
    readProjectFile("src/design-system/components.css"),
  ].join("\n");
  const requiredTokens = [
    "--lt-color-canvas",
    "--lt-color-hud-surface",
    "--lt-color-ready",
    "--lt-color-warning",
    "--lt-color-failed",
    "--lt-focus-color",
    "--lt-motion-normal",
    "--lt-hud-bg",
    "--lt-action-dock-height",
    "--lt-drawer-width",
    "--lt-status-strip-height",
  ];

  for (const token of requiredTokens) {
    assert.match(css, new RegExp(`${token}:`));
  }
  assert.match(css, /prefers-reduced-motion:\s*reduce/);
  assert.match(css, /--lt-color-hud-surface:\s*var\(--lt-alpha-ink-60\)/);
});

test("resolved lightfield and HUD colors pass normal-text contrast", () => {
  const declarations = designTokenDeclarations();
  const pairs = [
    ["--lt-color-accent-foreground", "--lt-color-accent"],
    ["--lt-color-accent-foreground", "--lt-color-accent-hover"],
    ["--lt-color-text", "--lt-color-canvas"],
    ["--lt-color-text-soft", "--lt-color-surface"],
    ["--lt-color-ready", "--lt-color-surface"],
    ["--lt-color-warning", "--lt-color-surface"],
    ["--lt-color-failed", "--lt-color-surface"],
    ["--lt-color-info", "--lt-color-surface"],
  ];

  for (const [foregroundToken, backgroundToken] of pairs) {
    const foreground = parseHexColor(resolveToken(declarations, foregroundToken));
    const background = parseHexColor(resolveToken(declarations, backgroundToken));
    assert.ok(
      contrastRatio(foreground, background) >= 4.5,
      `${foregroundToken} on ${backgroundToken} must meet WCAG AA normal-text contrast`,
    );
  }

  const hudSurface = resolveToken(declarations, "--lt-color-hud-surface").match(
    /^rgb\((\d+)\s+(\d+)\s+(\d+)\s*\/\s*([\d.]+)\)$/,
  );
  assert.ok(hudSurface);
  const alpha = Number(hudSurface[4]);
  const composite = hudSurface.slice(1, 4).map((channel) =>
    Math.round(Number(channel) * alpha + 255 * (1 - alpha)),
  ) as Rgb;
  const hudForeground = parseHexColor(
    resolveToken(declarations, "--lt-color-hud-text"),
  );
  assert.ok(
    contrastRatio(hudForeground, composite) >= 4.5,
    "HUD text must pass over its overlay on a worst-case white world",
  );
});

test("the application loads tokens before legacy styles", () => {
  const main = readProjectFile("src/main.tsx");
  const tokenImport = main.indexOf('import "./design-system/index.css";');
  const legacyImport = main.indexOf('import "./styles.css";');
  const lightfieldImport = main.indexOf('import "./lightfield.css";');

  assert.notEqual(tokenImport, -1);
  assert.notEqual(legacyImport, -1);
  assert.notEqual(lightfieldImport, -1);
  assert.ok(tokenImport < legacyImport);
  assert.ok(legacyImport < lightfieldImport);
});

test("lightfield is default and legacy aliases are explicitly scoped", () => {
  const styles = readProjectFile("src/styles.css");
  const app = readProjectFile("src/App.tsx");
  const root = styles.match(/:root\s*\{([\s\S]*?)\n\}/);
  const compatibility = styles.match(
    /\.legacy-workbench\[data-lt-surface="workbench"\]\s*\{([\s\S]*?)\n\}/,
  );
  assert.ok(root);
  assert.ok(compatibility);
  assert.match(root[1], /color-scheme:\s*light/);
  assert.doesNotMatch(root[1], /--bg:/);
  assert.match(app, /className=\{`legacy-workbench legacy-workbench--\$\{view\}`\}/);
  assert.match(app, /data-lt-surface="workbench"/);

  for (const alias of [
    "--bg",
    "--surface",
    "--text",
    "--accent",
    "--danger",
    "--topbar-height",
    "--sidebar-width",
  ]) {
    assert.match(compatibility[1], new RegExp(`${alias}:\\s*var\\(--lt-`));
  }
});

test("the design specification fixes world-first and engineering-truth rules", () => {
  const specification = readProjectFile("DESIGN_SYSTEM.md");

  assert.match(specification, /World context first/);
  assert.match(specification, /Progressive disclosure/);
  assert.match(specification, /Accessibility/);
  assert.match(specification, /Engineering-truth rules/);
  assert.match(specification, /qualified.*authoritative qualification evidence/i);
  assert.match(specification, /RobotSimUE Runtime\s+is the playable world/);
  assert.match(specification, /UE Create is the immersive 3D authoring surface/);
  assert.match(specification, /Web\s+preview.*not claim live UE rendering/is);
  assert.match(specification, /never claims to be the live UE world/is);
});

test("world forge uses a full-bleed image canvas with light floating chrome", () => {
  const css = readProjectFile("src/lightfield.css");

  assert.match(css, /\.lt-shell--forge\s*\{[\s\S]*?min-height:\s*100dvh/);
  assert.match(css, /\.world-forge__scene\s*\{[\s\S]*?background-image:\s*url\(/);
  assert.match(css, /\.world-forge__topbar/);
  assert.match(css, /\.world-forge__prompt/);
  assert.match(css, /\.world-forge__inspector/);
  assert.match(css, /\.world-forge__asset-tray/);
  assert.match(css, /backdrop-filter:\s*blur/);
});
