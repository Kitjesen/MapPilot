const scenarios = [
  ['details','工具详情展开','chat','工具调用','展开详情'],
  ['result','结果卡片出现','chat','地图 / 观测','返回结果'],
  ['queue','待执行队列重排','task','任务队列','提前一条'],
  ['approval','确认与执行反馈','task','审批卡片','允许执行'],
  ['send','发送与回答衔接','chat','对话输入','发送消息'],
  ['menu','菜单浮出','status','模型 / 权限','打开菜单'],
  ['tabs','标签与内容切换','status','设置 / 分析','切换标签'],
  ['notice','轻量完成通知','task','保存 / 录制','保存完成'],
  ['loading','加载到真实内容','chat','查询结果','读取数据'],
  ['metrics','数值变化','status','电量 / 统计','更新数值'],
  ['connection','连接恢复','status','设备状态','模拟重连'],
  ['inspect','诊断侧栏展开','status','关节 / 故障','查看详情'],
]
const variants = { A:{name:'轻量',description:'淡入 · 即时反馈',duration:180,easing:'cubic-bezier(.2,.8,.2,1)'}, B:{name:'连贯',description:'位移 · 空间联动',duration:360,easing:'cubic-bezier(.22,1,.36,1)'}, C:{name:'柔弹',description:'轻弹 · 柔和落位',duration:500,easing:'cubic-bezier(.34,1.35,.64,1)'} }
const gallery=document.querySelector('#gallery'), media=matchMedia('(prefers-reduced-motion: reduce)')
const scenes=new Map(); let choices={}, allRun=0, toastTimer
try { const saved=JSON.parse(localStorage.getItem('lingtu.motion-options.v1')||'{}'); for(const [id,v] of Object.entries(saved)) if(scenarios.some(s=>s[0]===id)&&variants[v]) choices[id]=v } catch {}
const dot='<span class="ico">✓</span>', bar='<div class="thin"></div>'
function initial(id){
  switch(id){
    case 'details': return `<div class="mini"><div class="line">${dot}<span>检查导航状态</span><span class="right muted">已返回 ⌄</span></div><div class="footline">下一步 · 核对巡检路线</div></div>`
    case 'result': return `<div class="line"><span class="ico">⌁</span><span>正在读取地图</span><span class="right muted">···</span></div><div class="footline">园区东区</div>`
    case 'queue': return `<div class="minilabel">待执行 3</div><div class="rows"><div class="queue-row" data-id="1"><span class="muted">01</span>检查地图<span class="handle">⠿</span></div><div class="queue-row" data-id="2"><span class="muted">02</span>读取定位<span class="handle">⠿</span></div><div class="queue-row emphasis" data-id="3"><span class="muted">03</span>查看右前腿故障<span class="handle">↑</span></div></div>`
    case 'approval': return `<div class="mini"><div class="line"><strong>保存地图副本</strong><span class="right pill">待确认</span></div><div class="bodytext">园区东区 → 东区_检查后</div>${bar}<div class="line"><span class="muted">保留原地图</span><span class="right approvalbutton">允许执行</span></div><div class="receipt"></div></div>`
    case 'send': return `<div class="bodytext">可以帮你检查地图与定位状态。</div><div class="posted"></div><div class="draft"><span>检查当前巡检条件</span><span class="sendcircle">↑</span></div><div class="answer"></div>`
    case 'menu': return `<div class="menu-anchor"><span class="tinybtn">默认权限 ⌄</span></div><div class="menu-space" style="height:115px"></div>`
    case 'tabs': return `<div class="tabs"><div class="tab-marker"></div><span>位置</span><span>姿态</span><span>关节</span></div><div class="tab-content">${[30,42,36,50,38,43,32].map(h=>`<div class="bar" style="height:${h}px"></div>`).join('')}</div><div class="footline">最近 30 秒</div>`
    case 'notice': return `<div class="line"><span class="ico">□</span><span>巡检记录</span><span class="right muted">正在保存</span></div><div class="notice-space" style="height:76px"></div>`
    case 'loading': return `<div class="mini"><div class="line"><span>机器人状态</span><span class="right muted">读取中</span></div><div class="loading-body"><div class="skeleton"></div><div class="skeleton"></div><div class="skeleton"></div></div></div>`
    case 'metrics': return `<div class="minilabel">本次巡检覆盖</div><div class="counter">44<small>%</small></div><div class="meter"><span></span></div><div class="footline">规划区域 · 园区东区</div>`
    case 'connection': return `<div style="text-align:center"><span class="statuslabel"><span class="dot" style="background:#c3ad8b"></span><span class="word">连接中断</span></span><div class="footline">等待设备重新上线</div></div>`
    case 'inspect': return `<div class="map-mini"><div class="map-line"></div><div class="pin"></div><span class="pill" style="position:absolute;left:12px;bottom:12px">右前腿 · 温度提醒</span></div>`
  }
}
function setVariant(s,v){s.controller?.abort();s.node.getAnimations({subtree:true}).forEach(a=>a.cancel());s.variant=v;delete s.node.dataset.played;s.node.dataset.variant=v;s.canvas.innerHTML=initial(s.id);s.play.disabled=false;s.play.textContent='▷ '+s.action;s.node.querySelectorAll('[data-v]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.v===v)));s.node.querySelector('.variant-name').textContent=variants[v].description;updateChosen(s)}
function updateChosen(s){s.node.classList.toggle('chosen',choices[s.id]===s.variant);s.node.querySelector('.choose').textContent=choices[s.id]===s.variant?'✓ 已选用':'选用 '+s.variant;s.node.querySelector('.choose').setAttribute('aria-pressed',String(choices[s.id]===s.variant))}
function toast(text){const el=document.querySelector('#toast');el.textContent=text;el.classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>el.classList.remove('show'),2200)}
function selectedText(){return 'LingTu 动效选择\n'+scenarios.filter(s=>choices[s[0]]).map(s=>`${s[1]}：${choices[s[0]]} · ${variants[choices[s[0]]].name}`).join('\n')}
function save(){try{localStorage.setItem('lingtu.motion-options.v1',JSON.stringify(choices))}catch{toast('浏览器无法保存，请复制选择清单')}document.querySelector('#selected-count').textContent=`已选 ${Object.keys(choices).length} / 12`;document.querySelector('#selection-hint').textContent=Object.keys(choices).length?'可混选 · 复制后发给我':'每个场景点「选用」';document.querySelector('#selection-text').value=selectedText()}
for(const [index,[id,title,category,tag,action]] of scenarios.entries()){
  const node=document.createElement('article');node.className='card';node.dataset.scene=id;node.dataset.category=category
  node.innerHTML=`<header class="cardheader"><h2><small>${String(index+1).padStart(2,'0')}</small>${title}</h2><span class="tag">${tag}</span></header><div class="stage"><div class="canvas"></div></div><footer class="cardfooter"><div class="variantbar" aria-label="${title}风格">${Object.keys(variants).map(v=>`<button data-v="${v}" aria-label="${title} ${v} ${variants[v].name}" aria-pressed="false">${v}</button>`).join('')}<span class="variant-name"></span></div><div class="actions"><button class="play" aria-label="播放${title}"></button><button class="choose" aria-label="选用${title}" aria-pressed="false"></button></div></footer>`
  gallery.append(node);const s={id,title,action,node,canvas:node.querySelector('.canvas'),play:node.querySelector('.play'),variant:choices[id]||'B'};scenes.set(id,s);setVariant(s,s.variant)
  node.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>{setVariant(s,b.dataset.v);void play(s)})
  s.play.onclick=()=>void play(s)
  node.querySelector('.choose').onclick=()=>{if(choices[id]===s.variant)delete choices[id];else choices[id]=s.variant;updateChosen(s);save()}
}
function sleep(r,ms){return new Promise((resolve,reject)=>{if(r.signal.aborted)return reject(new DOMException('Aborted','AbortError'));const timer=setTimeout(()=>{r.signal.removeEventListener('abort',cancel);resolve()},ms*r.speed);function cancel(){clearTimeout(timer);reject(new DOMException('Aborted','AbortError'))}r.signal.addEventListener('abort',cancel,{once:true})})}
async function animate(el,frames,r,extra={}){
  if(r.signal.aborted)throw new DOMException('Aborted','AbortError')
  if(r.reduced){for(const [key,value]of Object.entries(frames.at(-1)))if(!['offset','easing'].includes(key))el.style[key]=value;return}
  const a=el.animate(frames,{duration:r.duration*r.speed,easing:r.easing,fill:'forwards',...extra});const cancel=()=>a.cancel();r.signal.addEventListener('abort',cancel,{once:true});try{await a.finished}catch{if(r.signal.aborted)throw new DOMException('Aborted','AbortError')}finally{r.signal.removeEventListener('abort',cancel)}
}
function appear(el,r,axis='Y'){return animate(el,[{opacity:0,transform:r.v==='A'?'none':`translate${axis}(${r.v==='C'?18:10}px)`},{opacity:1,transform:`translate${axis}(0px)`}],r)}
async function run(s,r){
 const c=s.canvas,find=q=>c.querySelector(q)
 switch(s.id){
  case 'details':{
    const detail=document.createElement('div');detail.className='detail';detail.innerHTML='定位状态　就绪<br>地图版本　东区 v12<br>障碍物检查　已完成';const box=find('.mini');box.insertBefore(detail,find('.footline'));const h=detail.getBoundingClientRect().height
    if(r.v==='A')await animate(detail,[{opacity:0},{opacity:1}],r);else await animate(detail,[{height:'0px',paddingBlock:'0px',opacity:0},{height:h+'px',paddingBlock:'10px',opacity:1}],r)
    find('.right').textContent='已返回 ⌃';break
  }
  case 'result':{
    await sleep(r,280);find('.line').innerHTML=dot+'<span>地图检查完成</span><span class="right muted">已返回</span>';find('.footline').remove();const box=document.createElement('div');box.className='mini';box.style.marginTop='8px';box.innerHTML='<div class="resultname"><span>园区东区</span><span class="pill">可用</span></div><div style="display:flex;gap:8px"><div class="tile" style="flex:1"><strong>5</strong><small>cm 分辨率</small></div><div class="tile" style="flex:1"><strong>12</strong><small>巡检点</small></div></div>';c.append(box);if(r.v==='A')await appear(box,r);else{const h=box.offsetHeight;await animate(box,[{height:'0px',opacity:0,overflow:'hidden',transform:'translateY(8px)'},{height:h+'px',opacity:1,overflow:'hidden',transform:'translateY(0)'}],r)}break
  }
  case 'queue':{
    const parent=find('.rows'),rows=[...parent.children],old=rows.map(row=>row.getBoundingClientRect().top);parent.prepend(rows[2]);await Promise.all(rows.map((row,i)=>animate(row,r.v==='A'?[{opacity:.45},{opacity:1}]:[{transform:`translateY(${old[i]-row.getBoundingClientRect().top}px)`},{transform:'translateY(0px)'}],r)));rows[2].style.background='#f4f4f4';find('.minilabel').textContent='待执行 3 · 故障检查优先';break
  }
  case 'approval':{
    const b=find('.approvalbutton');b.style.minWidth='90px';b.innerHTML='<i class="status-ring"></i>确认中';await sleep(r,600);b.textContent='✓ 已允许';b.style.background='#efefef';b.style.color='#717171';find('.pill').textContent='等待执行';await appear(b,r);await sleep(r,500);find('.pill').textContent='已完成';find('.receipt').innerHTML='<div class="footline green">✓ 副本已保存，原地图保留</div>';await appear(find('.receipt'),r);break
  }
  case 'send':{
    find('.draft').innerHTML='<span class="muted">与机器人对话…</span><span class="sendcircle">↑</span>';find('.posted').innerHTML='<div class="sendbubble">检查当前巡检条件</div>';await appear(find('.sendbubble'),r);await sleep(r,400);find('.answer').innerHTML='<div class="bodytext" style="margin-top:12px">地图已就绪，正在核对定位。</div>';await appear(find('.answer'),r);break
  }
  case 'menu':{
    find('.menu-space').innerHTML='<div class="popover"><div>✓ 执行前确认</div><div>只读模式</div><div>Full access</div></div>';const box=find('.popover');box.style.transformOrigin='top right';await animate(box,r.v==='A'?[{opacity:0},{opacity:1}]:r.v==='B'?[{opacity:0,transform:'translateY(-8px)'},{opacity:1,transform:'translateY(0)'}]:[{opacity:0,transform:'scale(.94) translateY(-5px)'},{opacity:1,transform:'scale(1) translateY(0)'}],r);break
  }
  case 'tabs':{
    const marker=find('.tab-marker');await animate(marker,r.v==='A'?[{opacity:0,transform:'translateX(100%)'},{opacity:1,transform:'translateX(100%)'}]:[{transform:'translateX(0)'},{transform:'translateX(100%)'}],r);await Promise.all([...c.querySelectorAll('.bar')].map((el,i)=>animate(el,[{height:el.style.height},{height:[42,24,49,30,48,26,40][i]+'px',background:'#ababab'}],r,{delay:r.reduced?0:i*(r.v==='A'?0:25)})));break
  }
  case 'notice':{
    find('.right').textContent='保存完成';find('.notice-space').innerHTML='<div class="toastdemo" style="margin-top:14px"><span class="checkicon">✓</span><span>巡检记录已保存<br><small class="muted">今天 · 园区东区</small></span></div>';await appear(find('.toastdemo'),r,r.v==='C'?'X':'Y');break
  }
  case 'loading':{
    await sleep(r,300);const b=find('.loading-body');const h=b.offsetHeight;b.style.minHeight=h+'px';b.innerHTML='<div class="line"><span class="dot"></span>定位就绪<span class="right green">正常</span></div><div class="line"><span class="dot"></span>16 个电机在线<span class="right green">正常</span></div>';find('.right').textContent='实时';await Promise.all([...b.children].map((el,i)=>animate(el,[{opacity:0,transform:`translateY(${r.v==='A'?0:6}px)`},{opacity:1,transform:'translateY(0)'}],r,{delay:r.reduced?0:i*(r.v==='A'?0:80)})));break
  }
  case 'metrics':{
    const counter=find('.counter');counter.innerHTML='68<small>%</small>';await Promise.all([animate(counter,r.v==='A'?[{opacity:.3},{opacity:1}]:[{opacity:0,transform:'translateY(9px)'},{opacity:1,transform:'translateY(0)'}],r),animate(find('.meter span'),[{width:'44%'},{width:'68%'}],r)]);break
  }
  case 'connection':{
    find('.word').textContent='正在重连';find('.footline').textContent='设备重新上线后同步状态';await sleep(r,650);find('.word').textContent='已连接';find('.statuslabel').classList.add('online');find('.dot').style.background='#9c9c9c';find('.footline').textContent='状态已更新 · 不恢复旧运动指令';await appear(find('.statuslabel'),r);break
  }
  case 'inspect':{
    const pane=document.createElement('div');pane.className='side-mini';pane.innerHTML='<div class="line"><strong>右前腿</strong><span class="right muted">×</span></div><div class="line">髋关节<span class="right">52 °C</span></div><div class="line">膝关节<span class="right">43 °C</span></div><div class="footline">温度趋势 ↗</div>';find('.map-mini').append(pane);await animate(pane,r.v==='A'?[{opacity:0},{opacity:1}]:[{transform:'translateX(100%)',opacity:.5},{transform:'translateX(0)',opacity:1}],r);break
  }
 }
}
async function play(s){
 setVariant(s,s.variant);const controller=new AbortController();s.controller=controller;s.play.disabled=true;s.play.textContent='演示中…'
 const r={...variants[s.variant],v:s.variant,signal:controller.signal,reduced:media.matches||document.querySelector('#reduce').checked,speed:Number(document.querySelector('#speed').value)}
 try{await run(s,r);if(!r.signal.aborted)s.node.dataset.played='true'}catch(e){if(e.name!=='AbortError'){console.error(e);toast('演示未完成，请重试')}}finally{if(s.controller===controller){s.play.disabled=false;s.play.textContent='↻ 再看一次'}}
}
document.querySelectorAll('#directions [data-style]').forEach(b=>b.onclick=()=>{allRun++;document.querySelector('#play-all').textContent='依次播放全部 ↗';document.querySelectorAll('#directions button').forEach(el=>el.setAttribute('aria-pressed',String(el===b)));for(const s of scenes.values())setVariant(s,b.dataset.style)})
document.querySelectorAll('[data-filter]').forEach(b=>b.onclick=()=>{allRun++;document.querySelectorAll('[data-filter]').forEach(el=>el.setAttribute('aria-pressed',String(el===b)));for(const s of scenes.values()){s.node.hidden=b.dataset.filter!=='all'&&s.node.dataset.category!==b.dataset.filter;if(s.node.hidden)setVariant(s,s.variant)}document.querySelector('#play-all').textContent='依次播放全部 ↗'})
document.querySelector('#play-all').onclick=async()=>{const token=++allRun;const button=document.querySelector('#play-all');button.textContent='正在逐项播放…';for(const s of scenes.values()){if(token!==allRun)break;if(s.node.hidden)continue;await play(s)}if(token===allRun)button.textContent='全部重播 ↻'}
function resetMotion(){allRun++;for(const s of scenes.values())setVariant(s,s.variant);document.querySelector('#play-all').textContent='依次播放全部 ↗'}
document.querySelector('#reduce').onchange=resetMotion;media.addEventListener('change',resetMotion)
document.querySelector('#show-selection').onclick=()=>{document.querySelector('#selection-text').value=selectedText();document.querySelector('#selection-dialog').showModal()}
document.querySelector('#close-dialog').onclick=()=>document.querySelector('#selection-dialog').close()
document.querySelector('#copy').onclick=async()=>{if(!Object.keys(choices).length)return toast('先选一个喜欢的方案');try{await navigator.clipboard.writeText(selectedText());toast('选择已复制，发到对话里即可')}catch{document.querySelector('#selection-dialog').showModal();document.querySelector('#selection-text').value=selectedText();document.querySelector('#selection-text').select();toast('请手动复制清单')}}
save()
