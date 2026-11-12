import { createRequire } from 'node:module'
import assert from 'node:assert/strict'
import { fileURLToPath } from 'node:url'
const require=createRequire('C:/Users/99563/.codex/skills/stageshow/package.json')
const {chromium}=require('playwright-core')
const browser=await chromium.launch({channel:'chrome'})
const context=await browser.newContext({viewport:{width:1440,height:1000},permissions:['clipboard-read','clipboard-write']})
const page=await context.newPage(),errors=[]
page.on('pageerror',e=>errors.push(e.message))
try{
 await page.goto('http://127.0.0.1:47682/')
 await page.locator('.card').last().waitFor()
 assert.equal(await page.locator('.card').count(),12)
 await page.screenshot({path:fileURLToPath(new URL('overview.png',import.meta.url))})
 let checked=0
 for(const v of ['A','B','C']){
  for(const card of await page.locator('.card').all()){
   await card.locator(`[data-v="${v}"]`).click()
   await page.waitForFunction(id=>document.querySelector(`[data-scene="${id}"]`).dataset.played==='true',await card.getAttribute('data-scene'))
   checked++
  }
 }
 const details=page.locator('[data-scene="details"]')
 await details.locator('[data-v="B"]').click()
 await details.locator('[data-v="C"]').click()
 await page.waitForFunction(()=>document.querySelector('[data-scene="details"]').dataset.played==='true')
 await details.locator('.choose').click()
 await page.locator('#copy').click()
 assert.match(await page.evaluate(()=>navigator.clipboard.readText()),/工具详情展开：C/)
 await page.reload()
 assert.equal(await page.locator('#selected-count').textContent(),'已选 1 / 12')
 assert.equal(await details.getAttribute('data-variant'),'C')
 await page.getByRole('button',{name:'任务与审批',exact:true}).click()
 assert.equal(await page.locator('.card:visible').count(),3)
 await page.getByRole('button',{name:/全部 12/}).click()
 await page.emulateMedia({reducedMotion:'reduce'})
 await details.locator('[data-v="B"]').click()
 await page.waitForFunction(()=>document.querySelector('[data-scene="details"]').dataset.played==='true')
 assert.equal(await details.evaluate(el=>el.getAnimations({subtree:true}).filter(a=>a.playState==='running').length),0)
 await page.emulateMedia({reducedMotion:'no-preference'})
 await page.locator('#directions [data-style="B"]').click()
 for(const id of ['details','result','approval']){
  const c=page.locator(`[data-scene="${id}"]`);await c.locator('.play').click();await page.waitForFunction(id=>document.querySelector(`[data-scene="${id}"]`).dataset.played==='true',id)
 }
 await page.evaluate(()=>scrollTo(0,0))
 await page.screenshot({path:fileURLToPath(new URL('results.png',import.meta.url))})
 await page.setViewportSize({width:390,height:844})
 await page.screenshot({path:fileURLToPath(new URL('compact.png',import.meta.url))})
 assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth))
 assert.deepEqual(errors,[])
 console.log(JSON.stringify({checked,rapidSwitch:true,selectionPersists:true,clipboard:true,filters:true,reducedMotion:true,mobileOverflow:false,errors}))
}finally{await context.close();await browser.close()}
