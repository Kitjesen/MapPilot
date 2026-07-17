# LingTu Docs Web Guide

`web/guide/` is the static documentation surface built by the existing Vite
application. It intentionally has no connection to the live robot dashboard:
it reads Markdown into the build bundle, keeps search in the browser, and does
not make Gateway, teleoperation, goal, map, or MCP calls.

## Local preview

From `web/`:

```bash
npm run dev
```

Open `/guide/` on the Vite development server. Production builds emit the
separate entry at `dist/guide/index.html`, which the existing static Gateway
mount can serve at `/guide/`. Do not mount this surface at `/docs`; that route
is reserved for FastAPI's OpenAPI UI.

## Content model

`src/guide/docsRegistry.ts` is an explicit catalog, not a second source of
truth. It imports selected current Markdown files from `docs/` at build time,
defines their reading metadata, and drives the left navigation, local search,
page information, and previous/next links. Edit the corresponding Markdown
source first; then add it to the registry only when it belongs in the public
documentation path.

Plans, dated field runs, and archived material are intentionally not listed in
the main navigation. Link to them from a current page only when their status is
clear.

## Verification

```bash
npm run build
python -m pytest tests/docs/test_documentation_navigation.py -q
```

The build validates the isolated Vite entry and TypeScript code. The Python
test keeps the curated Markdown sources, local links, metadata, formatting, and
field-endpoint redaction intact.
