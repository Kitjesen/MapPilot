const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const {loadLibrary,createServer}=require('./server.cjs');
test('archived catalog exposes twelve assets with honest static and mechanism status',()=>{
  const library=loadLibrary();
  assert.equal(library.entries.length,12);
  assert.equal(new Set(library.entries.map(e=>e.id)).size,12);
  assert.equal(new Set(library.entries.map(e=>e.category)).size,4);
  for(const entry of library.entries){
    assert.equal(entry.capabilities.runnable,false);
    assert.equal(entry.qualification.physics_ready,false);
    assert.equal(entry.qualification.ue_imported,false);
    if(['boom_barrier','service_door'].includes(entry.id))assert.equal(entry.geometry.embedded_images,0);
    else assert.ok(entry.geometry.embedded_images>=3);
    if(entry.physics){assert.ok(fs.existsSync(library.routes.get(entry.physics)));assert.equal(entry.qualification.physics_model_tested,true);}
    for(const url of [entry.preview,entry.reference,entry.metadata,...entry.variants.map(v=>v.url)])assert.ok(fs.existsSync(library.routes.get(url)));
    assert.equal(fs.readFileSync(library.routes.get(entry.variants[0].url)).toString('ascii',0,4),'glTF');
  }
  assert.equal(library.entries.find(e=>e.id==='box_truck').variants.length,3);
  assert.equal(library.entries.filter(e=>e.new_in_batch).length,8);
  assert.equal(library.entries.filter(e=>e.physics).length,3);
  assert.equal(library.entries.filter(e=>e.fbx).length,3);
  const door=library.entries.find(e=>e.id==='service_door');
  assert.equal(door.version,'1.0.1');assert.ok(door.motionProject);assert.equal(door.variants.length,3);
  assert.equal(library.entries.filter(e=>e.source.license==='CC0-1.0').length,3);
});
test('local library serves catalog and models but does not expose arbitrary repo files',async()=>{
  const server=createServer();await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const base=`http://127.0.0.1:${server.address().port}`;
  try{
    const res=await fetch(base+'/library.json');assert.equal(res.status,200);assert.equal((await res.json()).entries.length,12);
    const head=await fetch(base+'/assets/fire_hydrant/model.glb',{method:'HEAD'});assert.equal(head.status,200);assert.equal(head.headers.get('content-type'),'model/gltf-binary');
    assert.equal((await fetch(base+'/AGENTS.md')).status,404);
    assert.equal((await fetch(base+'/library.json',{method:'POST'})).status,405);
  }finally{server.closeAllConnections();await new Promise(resolve=>server.close(resolve));}
});
