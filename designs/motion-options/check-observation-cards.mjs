import {createRequire} from 'node:module'
import assert from 'node:assert/strict'
const require=createRequire('C:/Users/99563/AppData/Local/npm-cache/_npx/e41f203b7505f1fb/node_modules/playwright/package.json')
const {chromium}=require('playwright')
const browser=await chromium.launch({channel:'chrome'})
const page=await browser.newPage({viewport:{width:1000,height:1000}})
const errors=[];page.on('pageerror',e=>errors.push(e.message))
try {
 await page.goto('http://127.0.0.1:47684/tests/observation-card-preview.html')
 await page.evaluate(()=>{window.__reads=[];const original=window.fetch;window.fetch=(input,init)=>{window.__reads.push({url:String(input),method:init?.method??'GET'});return original(input,init)}})
 for(const title of ['机器人状态','关节与故障','IMU 与记录','导航任务']){
  const card=page.getByRole('region',{name:title+'卡片',exact:true})
  const toggle=card.getByRole('button',{name:new RegExp(title+'.*展开')})
  await toggle.waitFor();await toggle.click()
  if(title==='机器人状态'||title==='导航任务') await card.getByText('已暂停',{exact:true}).waitFor()
  if(title==='关节与故障'){
   await card.getByText('16 个关节',{exact:true}).waitFor()
   await card.getByRole('button',{name:'电机与故障',exact:true}).click()
   await card.getByText('1 离线 · 1 有错误码',{exact:true}).waitFor()
  }
  if(title==='IMU 与记录') await card.getByText('正在加载…',{exact:true}).waitFor({state:'hidden'})
  await card.screenshot({path:'designs/motion-options/observation-'+({'机器人状态':'robot','关节与故障':'joints','IMU 与记录':'imu','导航任务':'navigation'}[title])+'.png'})
  await card.getByRole('button',{name:new RegExp(title+'.*收起')}).click()
  await page.waitForTimeout(250)
  const before=await page.evaluate(()=>window.__reads.filter(x=>x.url.includes('/state')||x.url.includes('/robot/')).length)
  await page.waitForTimeout(2200)
  assert.equal(await page.evaluate(()=>window.__reads.filter(x=>x.url.includes('/state')||x.url.includes('/robot/')).length),before,'collapse stops reads')
 }
 assert.deepEqual(await page.evaluate(()=>window.__reads.filter(x=>x.url.includes('/robot/')&&x.method!=='GET')),[])
 assert.deepEqual(errors,[])
 console.log('PASS four cards: expand reads, collapse stops polling, navigation states, 16 joints and fault receipt, no recording or motion writes')
}finally{await browser.close()}

