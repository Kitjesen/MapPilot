'use strict';
const $=id=>document.getElementById(id);
let entries=[],selected=null,filter='all';
const categoryNames={all:'全部素材',vehicle:'车辆',equipment:'设备',fire_safety:'消防',environment:'环境设施'};
function renderCategories(){
  $('categories').replaceChildren();
  for(const [key,label] of Object.entries(categoryNames)){
    const button=document.createElement('button');button.className='category-button';button.type='button';
    button.setAttribute('aria-pressed',String(filter===key));button.append(document.createTextNode(label));
    const count=document.createElement('span');count.textContent=entries.filter(e=>key==='all'||e.category===key).length;button.append(count);
    button.addEventListener('click',()=>{filter=key;renderCategories();renderCards();});$('categories').append(button);
  }
}
function renderCards(){
  const query=$('search').value.trim().toLocaleLowerCase();
  const shown=entries.filter(e=>(filter==='all'||e.category===filter)&&`${e.title} ${e.description} ${e.category_label}`.toLocaleLowerCase().includes(query));
  $('filter-title').textContent=categoryNames[filter];$('count').textContent=`${shown.length} 件`;$('empty').hidden=shown.length>0;$('cards').replaceChildren();
  for(const entry of shown){
    const button=document.createElement('button');button.type='button';button.className='asset-card';button.setAttribute('aria-pressed',String(selected?.id===entry.id));button.setAttribute('aria-label',`查看${entry.title}`);
    const image=document.createElement('img');image.src=entry.preview;image.alt='';button.append(image);
    if(entry.new_in_batch){const badge=document.createElement('span');badge.className='new';badge.textContent='新增';button.append(badge);}
    const title=document.createElement('strong');title.textContent=entry.title;button.append(title);
    const meta=document.createElement('small');meta.textContent=`${entry.category_label} · ${entry.variants.length>1?'视觉动作':'静态模型'}`;button.append(meta);
    button.addEventListener('click',()=>select(entry));$('cards').append(button);
  }
}
function select(entry){
  selected=entry;$('title').textContent=entry.title;$('category').textContent=entry.category_label;
  $('stage').textContent=entry.variants.length>1?'视觉动作 · 待接入':'静态素材 · 待接入';
  $('description').textContent=entry.description;$('triangles').textContent=entry.geometry.triangles.toLocaleString('en-US');
  $('source').textContent=`来源：${entry.source.provider}${entry.source.license?' · '+entry.source.license:''}`;
  for(const [id,url] of [['fbx',entry.fbx],['motion-project',entry.motionProject]]){const link=$(id);link.hidden=!url;if(url)link.href=url;}
  $('materials').textContent=`${entry.geometry.materials} / ${entry.geometry.embedded_images}`;$('version').textContent=`v${entry.version}`;
  $('scale').textContent=entry.scale_note;$('limitation').textContent=entry.qualification.physics_model_tested?'单体 MuJoCo 机构已测试；UE 联动与真实操控未验收。':'未完成 UE 导入、碰撞与交互验收。';
  $('reference').href=entry.reference;$('reference').textContent=entry.reference_label||'查看生成参考';$('physics').hidden=!entry.physics;if(entry.physics)$('physics').href=entry.physics;$('metadata').href=entry.metadata;$('variant').replaceChildren();
  entry.variants.forEach((v,i)=>{const option=document.createElement('option');option.value=String(i);option.textContent=v.label;$('variant').append(option);});
  $('variant-label').hidden=entry.variants.length<2;$('fallback').src=entry.preview;$('fallback').hidden=true;
  changeVariant();renderCards();
}
function changeVariant(){
  const variant=selected.variants[Number($('variant').value)||0];
  if(typeof $('model').pause==='function')$('model').pause();
  $('play').hidden=true;$('load-status').textContent='正在载入三维模型与贴图…';
  $('model').src=variant.url;$('model').alt=`${selected.title}三维预览`;$('download').href=variant.url;
  $('model').cameraOrbit='55deg 75deg 110%';$('model').cameraTarget='auto auto auto';
}
$('model').addEventListener('load',()=>{
  $('fallback').hidden=true;$('load-status').textContent='三维模型已加载';
  const animations=$('model').availableAnimations||[];$('play').hidden=animations.length===0;
  if(animations.length){$('model').animationName=animations[0];$('model').play();$('play').textContent='暂停动作';}
});
$('model').addEventListener('error',()=>{$('load-status').textContent='三维加载失败；显示预览图，仍可下载模型。';$('fallback').hidden=false;$('play').hidden=true;});
$('play').addEventListener('click',()=>{if($('model').paused){$('model').play();$('play').textContent='暂停动作';}else{$('model').pause();$('play').textContent='播放动作';}});
$('reset').addEventListener('click',()=>{$('model').cameraOrbit='55deg 75deg 110%';$('model').cameraTarget='auto auto auto';if(typeof $('model').jumpCameraToGoal==='function')$('model').jumpCameraToGoal();});
$('variant').addEventListener('change',changeVariant);$('search').addEventListener('input',renderCards);
fetch('/library.json').then(r=>{if(!r.ok)throw new Error('catalog');return r.json();}).then(data=>{
  entries=data.entries;$('total').textContent=`${entries.length} 件已归档`;
  renderCategories();if(entries.length)select(entries.filter(e=>e.new_in_batch).at(-1)||entries[0]);else{$('title').textContent='暂无素材';renderCards();}
}).catch(()=>{$('title').textContent='素材目录未能读取';$('load-status').textContent='请检查本地素材库服务是否已启动。';});
