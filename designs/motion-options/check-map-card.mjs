import {createRequire} from 'node:module'
import {fileURLToPath} from 'node:url'
import assert from 'node:assert/strict'
const require=createRequire('C:/Users/99563/AppData/Local/npm-cache/_npx/e41f203b7505f1fb/node_modules/playwright/package.json')
const {chromium}=require('playwright')
const browser=await chromium.launch({channel:'chrome',args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']})
const context=await browser.newContext({viewport:{width:1100,height:1000},recordVideo:{dir:fileURLToPath(new URL('map-video/',import.meta.url)),size:{width:1100,height:1000}}})
const page=await context.newPage(),errors=[]
page.on('pageerror',e=>errors.push(e.message))
try{
 await page.goto('http://127.0.0.1:47683/tests/conversation-style-preview.html')
 await page.evaluate(()=>{
   window.__mapCalls=[]
   const original=window.fetch
   window.fetch=async(input,init)=>{
     if(String(input).includes('/maps/')&&String(input).includes('/preview')){
       window.__mapCalls.push({url:String(input),method:init?.method||'GET'})
       const result=await original(input,init)
       if(window.__badEpoch){const body=await result.json();body.content_epoch++;return Response.json(body)}
       return result
     }
     return original(input,init)
   }
 })
 await page.getByRole('button',{name:'查看三维地图',exact:true}).click()
 const card=page.getByRole('region',{name:'三维地图 园区东区',exact:true})
 const canvas=card.locator('canvas').first()
 await canvas.waitFor()
 await canvas.scrollIntoViewIfNeeded()
 await page.waitForTimeout(300)
 const original=await canvas.screenshot()
 const box=await canvas.boundingBox()
 await page.mouse.move(box.x+box.width*.4,box.y+box.height*.55)
 await page.mouse.down();await page.mouse.move(box.x+box.width*.65,box.y+box.height*.65,{steps:15});await page.mouse.up()
 await page.waitForTimeout(250)
 assert.notDeepEqual(await canvas.screenshot(),original,'drag rotates actual rendered points')
 const rotated=await canvas.screenshot()
 await page.mouse.wheel(0,-220);await page.waitForTimeout(300)
 assert.notDeepEqual(await canvas.screenshot(),rotated,'wheel changes camera zoom')
 await card.getByRole('button',{name:'复位',exact:true}).click()
 await card.getByRole('button',{name:'截图',exact:true}).click()
 await card.getByRole('link',{name:'下载截图'}).waitFor()
 await page.screenshot({path:fileURLToPath(new URL('saved-map-3d.png',import.meta.url))})
 await card.getByRole('button',{name:'收起',exact:true}).click()
 assert.equal(await card.locator('canvas').count(),0)
 await card.getByRole('button',{name:'查看三维',exact:true}).click()
 await canvas.waitFor()
 assert.equal(await page.evaluate(()=>window.__mapCalls.length),1)
 await page.getByRole('button',{name:'查询地图结果',exact:true}).click()
 const library=page.getByRole('region',{name:'地图查询结果'})
 await library.waitFor()
 await library.locator('summary').filter({hasText:'办公区一层'}).click()
 const second=library.getByRole('region',{name:'三维地图 办公区一层'})
 await page.evaluate(()=>{window.__badEpoch=true})
 await second.getByRole('button',{name:'查看三维',exact:true}).click()
 await second.getByRole('alert').filter({hasText:'地图版本已变化'}).waitFor()
 assert.equal(await second.locator('canvas').count(),0)
 await page.evaluate(()=>{window.__badEpoch=false})
 await second.getByRole('button',{name:'重试',exact:true}).click()
 await second.locator('canvas').first().waitFor()
 await page.setViewportSize({width:390,height:844})
 await second.scrollIntoViewIfNeeded()
 assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth))
 await page.screenshot({path:fileURLToPath(new URL('saved-map-compact.png',import.meta.url))})
 assert.ok(await page.evaluate(()=>window.__mapCalls.every(call=>call.method==='GET')))
 assert.deepEqual(errors,[])
 console.log('PASS: tool auto-opens actual 3D renderer; rotate/zoom/reset/screenshot; close releases canvas; reopen uses cached data; list entry, version refusal/retry, 390px; map requests read-only. Geometry is synthetic fixture.')
}finally{await context.close();await browser.close()}
