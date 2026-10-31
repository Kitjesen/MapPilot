// Read-only local viewer over the existing asset-review catalog and archived files.
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');
const repoRoot = path.resolve(__dirname, '../../../..');
const mime = {'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.png':'image/png','.glb':'model/gltf-binary','.json':'application/json; charset=utf-8','.xml':'application/xml; charset=utf-8','.md':'text/plain; charset=utf-8'};

function loadLibrary(root = repoRoot) {
  const libraryRoot = path.join(root, 'sim/library/industrial');
  const catalog = JSON.parse(fs.readFileSync(path.join(libraryRoot, 'catalog.json'), 'utf8'));
  if (catalog.schema !== 'lingtu.sim.game-asset-review-catalog.v1') throw new Error('Unsupported review catalog');
  const routes = new Map();
  const entries = catalog.cards.map(card => {
    const model = card.evidence.find(e => ['source_model','conditioned_mesh'].includes(e.role));
    const folder = path.dirname(path.resolve(root, model.path));
    const record = JSON.parse(fs.readFileSync(path.join(folder, 'asset.json'), 'utf8'));
    if (record.id !== card.id) throw new Error(`Asset identity mismatch: ${card.id}`);
    function expose(file) {
      const resolved = path.resolve(folder, file);
      if (!resolved.startsWith(folder + path.sep) || !fs.statSync(resolved).isFile()) throw new Error(`Invalid library file: ${file}`);
      const url = `/assets/${card.id}/${encodeURIComponent(file)}`;
      routes.set(url, resolved);
      return url;
    }
    return {...record, review:card.review, capabilities:card.capabilities,
      preview:expose(record.preview), reference:expose(record.reference),
      variants:record.variants.map(v=>({...v,url:expose(v.file)})),
      metadata:expose('asset.json'),physics:record.physics?expose(record.physics):null,
      fbx:fs.existsSync(path.join(folder,'model.fbx'))?expose('model.fbx'):null,
      motionProject:fs.existsSync(path.join(folder,'editable-motion.zip'))?expose('editable-motion.zip'):null};
  });
  for (const file of ['index.html','app.js','style.css']) routes.set(file==='index.html'?'/':`/${file}`,path.join(__dirname,file));
  routes.set('/assets-ledger.md',path.join(root,'sim/library/ASSETS.md'));
  return {catalog, entries, routes};
}

function createServer(library = loadLibrary()) {
  return http.createServer((req,res)=>{
    if (req.method !== 'GET' && req.method !== 'HEAD') {res.writeHead(405);res.end();return;}
    const route = new URL(req.url,'http://localhost').pathname;
    if (route==='/library.json' || route==='/catalog.json') {
      const body=Buffer.from(JSON.stringify(route==='/library.json'?{title:library.catalog.title,entries:library.entries}:library.catalog));
      res.writeHead(200,{'Content-Type':mime['.json'],'Content-Length':body.length,'Cache-Control':'no-store'});
      res.end(req.method==='HEAD'?undefined:body);return;
    }
    const file=library.routes.get(route);
    if (!file) {res.writeHead(404);res.end('Not found');return;}
    try {
      const body=fs.readFileSync(file);
      res.writeHead(200,{'Content-Type':mime[path.extname(file)]||'application/octet-stream','Content-Length':body.length,'Cache-Control':'no-cache'});
      res.end(req.method==='HEAD'?undefined:body);
    } catch {res.writeHead(500);res.end('Library file unavailable');}
  });
}

if (require.main===module) {
  const server=createServer();
  server.listen(Number(process.argv[2]||63426),'127.0.0.1',()=>console.log(JSON.stringify({url:`http://127.0.0.1:${server.address().port}/`,library:'sim/library/industrial',mode:'read_only'})));
}
module.exports={loadLibrary,createServer};
