"""Generate a self-contained demo HTML for the LingTu map viewer (full feature set) and open it."""
import numpy as np, tempfile, os, subprocess, base64, json

rng = np.random.default_rng(42)
fx = rng.uniform(-8, 8, 3000); fy = rng.uniform(-6, 6, 3000); fz = rng.uniform(-0.05, 0.05, 3000)
wx = np.concatenate([rng.uniform(-8,-7.7,400), rng.uniform(7.7,8,400), rng.uniform(-8,8,400), rng.uniform(-8,8,400)])
wy = np.concatenate([rng.uniform(-6,6,400), rng.uniform(-6,6,400), rng.uniform(-6,-5.7,400), rng.uniform(5.7,6,400)])
wz = rng.uniform(0, 1.5, 1600)
pts = np.column_stack([np.concatenate([fx,wx]), np.concatenate([fy,wy]), np.concatenate([fz,wz])])

n = len(pts)
z = pts[:,2]; zmin, zmax = float(z.min()), float(z.max())
cx, cy = float(pts[:,0].mean()), float(pts[:,1].mean())
coords = ",".join(f"{pts[i,0]:.3f},{pts[i,1]:.3f},{pts[i,2]:.3f}" for i in range(n))
rx, ry, ryaw = 1.2, -0.8, 0.4

# Fake costmap
cm_cols = 60
cm_grid = np.zeros((cm_cols, cm_cols), dtype=np.uint8)
for r in range(cm_cols):
    for c in range(cm_cols):
        wall = (r<3 or r>cm_cols-4 or c<3 or c>cm_cols-4)
        obs  = ((r-20)**2+(c-15)**2<25) or ((r-35)**2+(c-40)**2<16)
        inf_ = ((r-20)**2+(c-15)**2<64) or ((r-35)**2+(c-40)**2<49)
        if wall or obs: cm_grid[r,c] = 100
        elif inf_:      cm_grid[r,c] = 40
cm_b64 = base64.b64encode(cm_grid.tobytes()).decode()

# Mock locations
mock_locs = json.dumps([
    {"name": "充电桩", "x": -5.2, "y": -4.1},
    {"name": "厨房",   "x":  3.8, "y":  4.5},
    {"name": "办公室", "x":  6.5, "y": -1.2},
    {"name": "门口",   "x": -2.0, "y": -5.5},
])

# Mock scene graph objects
mock_sg = json.dumps([
    {"label": "person",           "x":  2.1, "y":  1.4, "conf": 0.92},
    {"label": "chair",            "x": -1.5, "y":  3.2, "conf": 0.78},
    {"label": "charging_station", "x": -5.2, "y": -4.1, "conf": 0.99},
    {"label": "door",             "x":  5.0, "y":  0.0, "conf": 0.85},
])

html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>灵途 · LingTu — Demo</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0;}}
body{{background:#05060d;overflow:hidden;font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","Helvetica Neue",sans-serif;}}
.glass{{background:rgba(4,12,35,0.78);border:1px solid rgba(40,90,200,0.30);backdrop-filter:blur(22px);-webkit-backdrop-filter:blur(22px);border-radius:14px;}}
#hud{{position:fixed;top:16px;left:16px;padding:14px 18px;min-width:210px;z-index:50;}}
.lbl{{font-size:9px;letter-spacing:2.5px;color:rgba(80,140,255,0.55);text-transform:uppercase;margin-bottom:8px;}}
.row{{font-size:12.5px;color:#8ab4ff;letter-spacing:.4px;line-height:2.0;font-variant-numeric:tabular-nums;}}
.row b{{color:#5599ff;font-weight:600;}}
#ctrlBar{{position:fixed;bottom:22px;left:50%;transform:translateX(-50%);display:flex;align-items:center;gap:10px;padding:9px 16px;z-index:50;white-space:nowrap;}}
#missionTxt{{font-size:11.5px;color:rgba(100,155,255,0.8);letter-spacing:.5px;min-width:160px;}}
.sep{{width:1px;height:22px;background:rgba(50,100,200,0.22);}}
.btn{{background:rgba(20,50,130,0.55);border:1px solid rgba(55,110,240,0.45);color:#7aacff;font-size:10.5px;letter-spacing:.8px;padding:6px 15px;border-radius:8px;cursor:pointer;font-family:inherit;transition:all .16s;outline:none;}}
.btn:hover{{background:rgba(40,85,190,0.70);color:#c0d4ff;border-color:rgba(90,150,255,0.65);}}
.btn.stop{{background:rgba(110,20,20,0.55);border-color:rgba(220,50,50,0.45);color:#ff9999;}}
.btn.active{{background:rgba(120,20,20,0.55);border-color:rgba(220,60,60,0.50);color:#ffaaaa;}}
#hint{{position:fixed;top:16px;left:50%;transform:translateX(-50%);font-size:10px;color:rgba(50,100,200,0.42);letter-spacing:1.8px;pointer-events:none;z-index:40;}}
#toast{{position:fixed;bottom:78px;left:50%;transform:translateX(-50%);padding:9px 22px;font-size:11.5px;letter-spacing:.5px;display:none;pointer-events:none;z-index:200;color:#7ab8ff;}}
.dot{{width:6px;height:6px;border-radius:50%;background:#44cc88;box-shadow:0 0 7px #44cc8899;display:inline-block;margin-right:8px;flex-shrink:0;}}
#camPanel{{position:fixed;bottom:22px;right:16px;padding:12px 14px;z-index:50;}}
#camPanel img{{width:216px;height:122px;object-fit:cover;border-radius:8px;background:#060c20;display:block;}}
#camStatus{{font-size:9px;color:rgba(80,140,255,0.50);margin-top:5px;letter-spacing:1px;text-align:center;}}
#locPanel{{position:fixed;top:16px;right:16px;padding:12px 14px;z-index:50;min-width:138px;}}
#locBtns{{display:flex;flex-direction:column;gap:5px;margin-top:2px;}}
#kbHud{{position:fixed;bottom:80px;right:16px;display:grid;grid-template-columns:repeat(3,22px);grid-template-rows:repeat(2,22px);gap:3px;opacity:0;transition:opacity .3s;z-index:60;}}
.kk{{width:22px;height:22px;border-radius:4px;background:rgba(15,40,110,0.45);border:1px solid rgba(50,100,220,0.28);font-size:9px;color:rgba(80,140,220,0.55);display:flex;align-items:center;justify-content:center;}}
.kk.on{{background:rgba(40,90,200,0.75);border-color:rgba(100,160,255,0.75);color:#c8dcff;box-shadow:0 0 6px rgba(80,140,255,0.4);}}
</style>
</head>
<body>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>

<div id="hud" class="glass">
  <div class="lbl">灵途 · 导航状态 [DEMO]</div>
  <div class="row">X&nbsp;<b id="hx">{rx:.2f}</b>&nbsp;&nbsp;Y&nbsp;<b id="hy">{ry:.2f}</b></div>
  <div class="row">模式&nbsp;<b id="hmode">演示</b></div>
  <div class="row" id="hmission" style="font-size:11px;color:rgba(80,130,220,0.60);">机器人未连接</div>
</div>

<div id="ctrlBar" class="glass">
  <span class="dot" id="dot"></span>
  <span id="missionTxt">待机</span>
  <div class="sep"></div>
  <button class="btn stop" onclick="showToast('■ 紧急停止 [Demo]')">■&nbsp;停止</button>
  <button class="btn" id="expBtn" onclick="toggleExplore()">▶&nbsp;探索</button>
</div>

<div id="hint">单击地图发送导航目标 · 拖拽旋转 · 滚轮缩放 · WASD手动驾驶</div>
<div id="toast" class="glass"></div>

<!-- Camera panel (demo: gray placeholder) -->
<div id="camPanel" class="glass">
  <div class="lbl">摄像头</div>
  <img id="camImg" alt="">
  <div id="camStatus">演示模式</div>
</div>

<!-- Location shortcuts (populated from mock data) -->
<div id="locPanel" class="glass">
  <div class="lbl">快速导航</div>
  <div id="locBtns"></div>
</div>

<!-- WASD keyboard indicator -->
<div id="kbHud">
  <div></div><div class="kk" id="kW">W</div><div></div>
  <div class="kk" id="kA">A</div><div class="kk" id="kS">S</div><div class="kk" id="kD">D</div>
</div>

<script>
const scene=new THREE.Scene(); scene.background=new THREE.Color(0x05060d);
scene.fog=new THREE.FogExp2(0x05060d,0.0055);
const camera=new THREE.PerspectiveCamera(50,innerWidth/innerHeight,0.1,600);
camera.position.set({cx+5:.1f},{cy-28:.1f},28); camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({{antialias:true}});
renderer.setSize(innerWidth,innerHeight); renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.toneMapping=THREE.ACESFilmicToneMapping; renderer.toneMappingExposure=1.1;
document.body.appendChild(renderer.domElement);
const controls=new THREE.OrbitControls(camera,renderer.domElement);
controls.target.set({cx:.1f},{cy:.1f},1.5);
controls.enableDamping=true; controls.dampingFactor=0.06;
controls.minDistance=2; controls.maxDistance=250; controls.update();

scene.add(new THREE.AmbientLight(0x1a2a60,2.5));
const dl=new THREE.DirectionalLight(0x5588ff,2.0); dl.position.set(4,-6,10); scene.add(dl);

// Point cloud — cold blue
(function(){{
  const geo=new THREE.BufferGeometry();
  const pos=new Float32Array({n*3}); const col=new Float32Array({n*3});
  const pts=[{coords}];
  const zmin={zmin:.3f},zmax={zmax:.3f},zr=zmax-zmin||1;
  for(let i=0;i<{n};i++){{
    pos[i*3]=pts[i*3];pos[i*3+1]=pts[i*3+1];pos[i*3+2]=pts[i*3+2];
    const t=(pts[i*3+2]-zmin)/zr;
    if(t<0.5){{const s=t*2;col[i*3]=0.04*(1-s);col[i*3+1]=0.10*(1-s)+0.33*s;col[i*3+2]=0.24*(1-s)+0.78*s;}}
    else{{const s=(t-0.5)*2;col[i*3]=0;col[i*3+1]=0.33*(1-s)+0.90*s;col[i*3+2]=0.78*(1-s)+1.0*s;}}
  }}
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geo.setAttribute('color',new THREE.BufferAttribute(col,3));
  scene.add(new THREE.Points(geo,new THREE.PointsMaterial({{size:0.035,vertexColors:true,sizeAttenuation:true,transparent:true,opacity:0.88}})));
}})();

const grid=new THREE.GridHelper(100,100,0x0c1830,0x070e1d);
grid.rotation.x=Math.PI/2; grid.position.set({cx:.1f},{cy:.1f},{zmin:.2f}-0.01); scene.add(grid);

// Robot
const robotGroup=new THREE.Group();
robotGroup.position.set({rx:.3f},{ry:.3f},0); robotGroup.rotation.z={ryaw:.4f};
scene.add(robotGroup);
const fb=new THREE.Mesh(new THREE.BoxGeometry(0.62,0.36,0.18),
  new THREE.MeshBasicMaterial({{color:0x2255cc,transparent:true,opacity:0.55,wireframe:true}}));
fb.position.set(0,0,0.40); robotGroup.add(fb);
if(typeof THREE.STLLoader!=='undefined'){{
  const loader=new THREE.STLLoader();
  const mb=new THREE.MeshPhongMaterial({{color:0xb8cdff,specular:0x4466bb,shininess:55,transparent:true,opacity:0.92}});
  const ml=new THREE.MeshPhongMaterial({{color:0x8aaae8,specular:0x223388,shininess:38,transparent:true,opacity:0.88}});
  function load(url,mat,px,py,pz){{loader.load(url,function(g){{g.computeVertexNormals();const m=new THREE.Mesh(g,mat.clone());m.position.set(px,py,pz);robotGroup.add(m);fb.visible=false;}},undefined,()=>{{}});}}
  const bz=0.40;
  load('/robot/meshes/base_link.STL',mb,0,0,bz);
  load('/robot/meshes/fr_hip_Link.STL',ml,0.2395,-0.0646,bz+0.0807);
  load('/robot/meshes/fl_hip_Link.STL',ml,0.2395,0.0654,bz+0.0807);
  load('/robot/meshes/rr_hip_Link.STL',ml,-0.2395,-0.0662,bz+0.0807);
  load('/robot/meshes/rl_hip_Link.STL',ml,-0.2395,0.0638,bz+0.0807);
}}

const ringMat=new THREE.MeshBasicMaterial({{color:0x3377ff,transparent:true,opacity:0.55,side:THREE.DoubleSide}});
const ringMesh=new THREE.Mesh(new THREE.RingGeometry(0.38,0.52,56),ringMat);
ringMesh.rotation.x=Math.PI/2; ringMesh.position.set({rx:.3f},{ry:.3f},0.015); scene.add(ringMesh);

// Goal
const goalGroup=new THREE.Group(); goalGroup.visible=false;
const gS=new THREE.Mesh(new THREE.SphereGeometry(0.13,20,20),new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.88}}));
gS.position.z=0.13; goalGroup.add(gS);
const gR1=new THREE.Mesh(new THREE.RingGeometry(0.18,0.30,48),new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.55,side:THREE.DoubleSide}}));
gR1.rotation.x=Math.PI/2; goalGroup.add(gR1);
const gR2=new THREE.Mesh(new THREE.RingGeometry(0.34,0.46,48),new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.22,side:THREE.DoubleSide}}));
gR2.rotation.x=Math.PI/2; goalGroup.add(gR2);
scene.add(goalGroup);

// Trajectory (mock spiral)
const TM=2000; const tPos=new Float32Array(TM*3); const tCol=new Float32Array(TM*3);
const tGeo=new THREE.BufferGeometry();
tGeo.setAttribute('position',new THREE.BufferAttribute(tPos,3));
tGeo.setAttribute('color',new THREE.BufferAttribute(tCol,3));
tGeo.setDrawRange(0,0);
scene.add(new THREE.Line(tGeo,new THREE.LineBasicMaterial({{vertexColors:true,transparent:true,opacity:0.9}})));
let tN=0;
(function(){{
  for(let i=0;i<300;i++){{const tt=i/300*Math.PI*4;tPos[tN*3]={cx:.1f}+Math.cos(tt)*3.5;tPos[tN*3+1]={cy:.1f}+Math.sin(tt)*2;tPos[tN*3+2]=0.08;tN++;}}
  const nn=Math.min(tN,TM);
  for(let i=0;i<nn;i++){{const f=i/nn;tCol[i*3]=0.04+0.22*f;tCol[i*3+1]=0.20+0.47*f;tCol[i*3+2]=0.80+0.20*f;}}
  tGeo.attributes.position.needsUpdate=true;tGeo.attributes.color.needsUpdate=true;tGeo.setDrawRange(0,tN);
}})();

// Planned path
const pathGeo=new THREE.BufferGeometry();
(function(){{const a=[];for(let i=0;i<25;i++){{const t=i/24;a.push({rx:.2f}+t*5,{ry:.2f}+t*3,0.05);}}pathGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(a),3));}})();
scene.add(new THREE.Line(pathGeo,new THREE.LineBasicMaterial({{color:0x00ffaa,transparent:true,opacity:0.72}})));

// Costmap
(function(){{
  const cols={cm_cols},res=0.2,ox=-6.0,oy=-5.0,sz=cols*res;
  const canvas=document.createElement('canvas');canvas.width=cols;canvas.height=cols;
  const ctx=canvas.getContext('2d');
  const bin=atob('{cm_b64}');const arr=new Uint8Array(bin.length);
  for(let i=0;i<bin.length;i++)arr[i]=bin.charCodeAt(i);
  const img=ctx.createImageData(cols,cols);
  for(let i=0;i<cols*cols;i++){{const v=arr[i];if(v<=0){{img.data[i*4+3]=0;continue;}}img.data[i*4]=v>50?215:175;img.data[i*4+1]=v>50?42:85;img.data[i*4+2]=28;img.data[i*4+3]=Math.min(205,v*2+55);}}
  ctx.putImageData(img,0,0);const tex=new THREE.CanvasTexture(canvas);
  const mesh=new THREE.Mesh(new THREE.PlaneGeometry(sz,sz),new THREE.MeshBasicMaterial({{map:tex,transparent:true,opacity:0.50,side:THREE.DoubleSide,depthWrite:false}}));
  mesh.position.set(ox+sz/2,oy+sz/2,0.01);scene.add(mesh);
}})();

// Scene graph object markers
const _sgColors={{person:0xff4455,chair:0x44aaff,door:0xffcc00,table:0x88ddff,charging_station:0x00ff88,dog:0xff8844}};
function _sgUpdate(objects){{
  objects.forEach(function(obj){{
    const color=_sgColors[obj.label]||0xaaaaff;
    const s=new THREE.Mesh(new THREE.SphereGeometry(0.10,10,10),new THREE.MeshBasicMaterial({{color,transparent:true,opacity:0.45+0.45*obj.conf}}));
    s.position.set(obj.x,obj.y,0.55);scene.add(s);
    const lg=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(obj.x,obj.y,0.05),new THREE.Vector3(obj.x,obj.y,0.45)]);
    scene.add(new THREE.Line(lg,new THREE.LineBasicMaterial({{color,transparent:true,opacity:0.35}})));
  }});
}}
_sgUpdate({mock_sg});

// UI helpers
const $toast=document.getElementById('toast');
const $mission=document.getElementById('missionTxt');
const $expBtn=document.getElementById('expBtn');
let _exploring=false;
function showToast(msg,ms){{$toast.textContent=msg;$toast.style.display='block';clearTimeout($toast._t);$toast._t=setTimeout(()=>{{$toast.style.display='none';}},ms||3000);}}
function toggleExplore(){{_exploring=!_exploring;$expBtn.textContent=_exploring?'■ 停止探索':'▶ 探索';$expBtn.className=_exploring?'btn active':'btn';$mission.textContent=_exploring?'自主探索中...':'待机';showToast(_exploring?'自主探索启动 [Demo]':'探索已停止 [Demo]');}}

// Click-to-navigate
const ray=new THREE.Raycaster();const mv=new THREE.Vector2();const gp=new THREE.Plane(new THREE.Vector3(0,0,1),0);let _md=null;
renderer.domElement.addEventListener('mousedown',e=>{{_md={{x:e.clientX,y:e.clientY}};}});
renderer.domElement.addEventListener('mouseup',e=>{{
  if(!_md)return;const dx=e.clientX-_md.x,dy=e.clientY-_md.y;_md=null;
  if(dx*dx+dy*dy>25)return;
  mv.x=(e.clientX/innerWidth)*2-1;mv.y=-(e.clientY/innerHeight)*2+1;
  ray.setFromCamera(mv,camera);const tgt=new THREE.Vector3();
  if(!ray.ray.intersectPlane(gp,tgt))return;
  goalGroup.position.set(tgt.x,tgt.y,0);goalGroup.visible=true;
  showToast('导航目标 → ('+tgt.x.toFixed(2)+', '+tgt.y.toFixed(2)+')  [Demo]');
  $mission.textContent='导航中 → ('+tgt.x.toFixed(1)+', '+tgt.y.toFixed(1)+')';
}});

// Camera panel (demo: canvas placeholder with animated scan line)
(function(){{
  const img=document.getElementById('camImg');
  const c=document.createElement('canvas');c.width=216;c.height=122;
  const ctx2=c.getContext('2d');let scanY=0;
  function drawCam(){{
    ctx2.fillStyle='#060c20';ctx2.fillRect(0,0,216,122);
    ctx2.strokeStyle='rgba(40,80,200,0.15)';ctx2.lineWidth=1;
    for(let i=0;i<122;i+=8){{ctx2.beginPath();ctx2.moveTo(0,i);ctx2.lineTo(216,i);ctx2.stroke();}}
    ctx2.fillStyle='rgba(0,180,255,0.12)';ctx2.fillRect(0,scanY,216,3);
    scanY=(scanY+1)%122;
    ctx2.font='10px monospace';ctx2.fillStyle='rgba(80,140,255,0.45)';
    ctx2.fillText('CAMERA OFFLINE [DEMO]',18,64);
    img.src=c.toDataURL();
    setTimeout(drawCam,40);
  }}
  drawCam();
  document.getElementById('camStatus').textContent='演示模式 — 无摄像头信号';
}})();

// Location shortcuts (from mock data)
(function(){{
  const locs={mock_locs};
  const btns=document.getElementById('locBtns');
  locs.forEach(function(loc){{
    const b=document.createElement('button');
    b.className='btn';b.style.cssText='text-align:left;width:100%;font-size:10px;padding:5px 10px;';
    b.textContent='→ '+loc.name;
    b.onclick=function(){{showToast('导航 → '+loc.name+'  [Demo]');$mission.textContent='导航中 → '+loc.name;}};
    btns.appendChild(b);
    // Pin on map
    const pin=new THREE.Mesh(new THREE.ConeGeometry(0.12,0.35,6),new THREE.MeshBasicMaterial({{color:0x00ffaa,transparent:true,opacity:0.78}}));
    pin.rotation.x=Math.PI;pin.position.set(loc.x,loc.y,0.35);scene.add(pin);
    const pRing=new THREE.Mesh(new THREE.RingGeometry(0.14,0.22,32),new THREE.MeshBasicMaterial({{color:0x00ffaa,transparent:true,opacity:0.32,side:THREE.DoubleSide}}));
    pRing.rotation.x=Math.PI/2;pRing.position.set(loc.x,loc.y,0.015);scene.add(pRing);
  }});
}})();

// WASD keyboard control
(function(){{
  const kbHud=document.getElementById('kbHud');
  const kW=document.getElementById('kW'),kA=document.getElementById('kA'),kS=document.getElementById('kS'),kD=document.getElementById('kD');
  const _k={{}};let _wasMoving=false;
  document.addEventListener('keydown',function(e){{const k=e.key.toLowerCase();if(['w','a','s','d'].includes(k)){{e.preventDefault();_k[k]=true;kbHud.style.opacity='1';}}}});
  document.addEventListener('keyup',function(e){{_k[e.key.toLowerCase()]=false;}});
  setInterval(function(){{
    kW.classList.toggle('on',!!_k['w']);kA.classList.toggle('on',!!_k['a']);kS.classList.toggle('on',!!_k['s']);kD.classList.toggle('on',!!_k['d']);
    const vx=(_k['w']?0.4:0)+(_k['s']?-0.4:0);const wz=(_k['a']?0.7:0)+(_k['d']?-0.7:0);
    if(vx!==0||wz!==0){{
      // Demo: just show toast once
      if(!_wasMoving)showToast('手动驾驶 vx='+vx.toFixed(1)+' wz='+wz.toFixed(1)+'  [Demo]',1500);
      _wasMoving=true;
    }} else if(_wasMoving){{_wasMoving=false;setTimeout(function(){{kbHud.style.opacity='0';}},1800);}};
  }},100);
}})();

// Animation loop
let _t=0;
(function _loop(){{
  requestAnimationFrame(_loop);_t+=0.016;
  ringMat.opacity=0.35+0.25*(0.5+0.5*Math.sin(_t*2.8));
  ringMesh.scale.setScalar(1.0+0.07*Math.sin(_t*2.8));
  robotGroup.position.x={rx:.3f}+0.008*Math.sin(_t*0.7);
  robotGroup.position.y={ry:.3f}+0.008*Math.cos(_t*0.5);
  ringMesh.position.x=robotGroup.position.x;ringMesh.position.y=robotGroup.position.y;
  if(goalGroup.visible){{const gpp=0.5+0.5*Math.sin(_t*4.5);gR1.material.opacity=0.28+0.27*gpp;gR2.scale.setScalar(1.0+0.18*gpp);gR2.material.opacity=0.18*(1-gpp);}}
  controls.update();renderer.render(scene,camera);
}})();
addEventListener('resize',()=>{{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);}});
</script>
</body>
</html>"""

tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.html', mode='w', encoding='utf-8')
tmp.write(html)
tmp.close()
print(f"Generated: {tmp.name}")
subprocess.Popen(['cmd', '/c', 'start', '', tmp.name])
