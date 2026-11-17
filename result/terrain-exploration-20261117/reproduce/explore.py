"""Fixed analytic terrains, voxelized OctoMaps and production global-planner probes."""
import argparse
import json
import math
import os
from pathlib import Path
import runpy
import subprocess
import time

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
BIN = ROOT / 'build/mls-stairs-fix-20261116/native/Release'
FLAGS = runpy.run_path(str(ROOT/'tools/bench/mls_regression.py'))['VERIFY_OPTIONS']
R = .05
OPTIONS = json.loads((OUT/'options.json').read_text())
os.environ['PATH'] = str(BIN)+';C:/opt/octomap-1.10.0-msvc-x64/bin;'+os.environ['PATH']


def write(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def designs():
    cases=[]
    def add(name,title,fn,*,length=8,width=2.4,feature='',expected='probe',diagonal=False):
        cases.append(dict(id=name,title=title,fn=fn,length=length,width=width,
                          feature=feature,expected=expected,diagonal=diagonal))
    def wave(amplitude,wavelength):
        return lambda x,y: amplitude*np.sin(2*np.pi*x/wavelength)+np.zeros_like(y)
    for a,period in [(.1,4),(.25,4),(.4,4),(.15,1.2),(.1,.6)]:
        add(f'wave_{round(a*100):02}_{round(period*100):03}',
            f'波浪：振幅 {a} m / 波长 {period} m',wave(a,period))
    add('wave_long','长距离波浪：20 m',wave(.2,4),length=20,width=3)
    add('wave_narrow','波浪：1.2 m 窄通道',wave(.25,4),width=1.2)
    add('wave_box','波浪坡面绕箱体',wave(.25,4),feature='box')
    add('wave_gap','波浪途中 0.5 m 全宽断口',wave(.25,4),feature='gap',expected='blocked')
    add('wave_ceiling','波浪局部低顶棚',wave(.1,4),feature='ceiling',expected='blocked')
    for angle in (15,25,35,45):
        add(f'cross_{angle}',f'横坡 {angle}°：沿等高线行走',
            lambda x,y,a=angle: np.tan(np.radians(a))*y+np.zeros_like(x))
    add('diagonal_20','斜向穿越 20° 平面',lambda x,y: np.tan(np.radians(20))*x+np.zeros_like(y),diagonal=True)
    add('compound_plane','纵坡 20° + 横坡 15°',lambda x,y:np.tan(np.radians(20))*x+np.tan(np.radians(15))*y,diagonal=True)
    for angle in (20,30):
        slope=np.tan(np.radians(angle))
        add(f'crest_{angle}',f'尖坡顶过渡 {angle}°',lambda x,y,s=slope:s*(4-np.abs(x-4))+np.zeros_like(y))
        add(f'valley_{angle}',f'V 形坡脚过渡 {angle}°',lambda x,y,s=slope:s*np.abs(x-4)+np.zeros_like(y))
    add('ramp_wave','10° 坡面叠加波浪',lambda x,y:np.tan(np.radians(10))*x+.12*np.sin(np.pi*x)+np.zeros_like(y))
    add('ripple_2d','双向起伏表面',lambda x,y:.15*np.sin(2*np.pi*x/3)*np.cos(np.pi*y/2),diagonal=True)
    add('saddle','鞍形地形',lambda x,y:.03*(x-4)**2-.12*y**2,diagonal=True)
    add('bank_transition','左右横坡逐渐反转',lambda x,y:.4*y*np.sin(2*np.pi*x/8),diagonal=True)
    add('wave_upper','波浪下层与无连接上层',wave(.1,4),feature='upper',expected='blocked')
    for angle in (55,65):
        add(f'cross_{angle}',f'横坡 {angle}°：沿等高线行走',
            lambda x,y,a=angle: np.tan(np.radians(a))*y+np.zeros_like(x))
    add('wave_35_120','短波深起伏：振幅 0.35 m / 波长 1.2 m',wave(.35,1.2))
    add('wave_50_200','大起伏：振幅 0.50 m / 波长 2.0 m',wave(.5,2))
    add('chicane_flat','平地连续 S 弯',lambda x,y:np.zeros_like(x+y),width=3,feature='chicane')
    add('chicane_ramp','20° 坡面连续 S 弯',lambda x,y:np.tan(np.radians(20))*x+np.zeros_like(y),width=3,feature='chicane')
    for width in (.4,.2):
        add(f'ridge_{round(width*100)}',f'{width} m 宽窄脊，两侧低 1 m',
            lambda x,y,w=width:np.where((abs(y)<w/2)|(x<1.5)|(x>6.5),0.,-1.))
    add('wave_phase','波浪平移半个体素',lambda x,y:.25*np.sin(2*np.pi*(x+.025)/4)+np.zeros_like(y))
    for fraction in (10,30):
        add(f'missing_{fraction}',f'波浪地面随机缺测 {fraction}%',wave(.25,4),feature=f'missing_{fraction}')
    return cases


def generate(case):
    folder=OUT/'maps';folder.mkdir(exist_ok=True)
    x=(np.arange(round(case['length']/R))+.5)*R
    y=(np.arange(round(case['width']/R))+.5)*R-case['width']/2
    xx,yy=np.meshgrid(x,y,indexing='ij')
    ideal=case['fn'](xx,yy)
    heights=np.round(ideal/R)*R
    mask=np.ones_like(heights,dtype=bool)
    if case['feature']=='gap':mask[(xx>=4)&(xx<4.5)]=False
    if case['feature'].startswith('missing_'):
        fraction=int(case['feature'].split('_')[1])/100
        mask=(np.random.default_rng(20261117).random(heights.shape)>=fraction)|(xx<1.5)|(xx>6.5)
    bottom=math.floor(float(heights.min())/R)*R-.1
    boxes=[]
    for i,j in zip(*np.where(mask)):
        boxes.append([round(x[i]-R/2,8),round(y[j]-R/2,8),round(bottom,8),
                      round(x[i]+R/2,8),round(y[j]+R/2,8),round(float(heights[i,j]),8)])
    extra=[]
    if case['feature']=='box':
        top=float(heights[(xx>=3.7)&(xx<4.3)&(abs(yy)<.35)].max())
        extra.append(dict(kind='obstacle',min=[3.7,-.35,bottom],max=[4.3,.35,top+.9]))
    if case['feature']=='ceiling':
        # A horizontal roof over the crest leaves < 0.65 m of local headroom.
        top=float(heights[(xx>=2.5)&(xx<5.5)].max())+.6
        extra.append(dict(kind='ceiling',min=[2.5,-case['width']/2,top],max=[5.5,case['width']/2,top+.1]))
    if case['feature']=='upper':
        top=math.ceil((float(heights.max())+1.2)/R)*R
        extra.append(dict(kind='upper',min=[0,-case['width']/2,top-.1],max=[case['length'],case['width']/2,top]))
    if case['feature']=='chicane':
        top=float(heights.max())+1.2
        extra.extend([dict(kind='obstacle',min=[2,-1.5,bottom],max=[2.2,.5,top]),
                      dict(kind='obstacle',min=[5,-.5,bottom],max=[5.2,1.5,top])])
    boxes.extend(e['min']+e['max'] for e in extra)
    box_path=folder/f"{case['id']}.boxes.txt"
    box_path.write_text('\n'.join(' '.join(map(str,b)) for b in boxes),encoding='utf-8')
    map_path=folder/f"{case['id']}.bt"
    made=subprocess.run([str(OUT/'native/Release/make_scenes.exe'),str(box_path),str(map_path),os.devnull],capture_output=True,text=True,check=True)
    dx,dy=np.gradient(ideal,R,R)
    def point(px,py):
        i=min(len(x)-1,max(0,int(px/R)));j=min(len(y)-1,max(0,int((py+case['width']/2)/R)))
        return [float(x[i]),float(y[j]),float(heights[i,j]+OPTIONS['support_height_m'])]
    pairs=[('along',point(.75,.025),point(case['length']-.75,.025))]
    if case['diagonal']:
        pairs.append(('diagonal',point(.75,-case['width']/3),point(case['length']-.75,case['width']/3)))
    if case['feature']=='upper':pairs=[('upper',pairs[0][1],[pairs[0][2][0],pairs[0][2][1],extra[0]['max'][2]+OPTIONS['support_height_m']])]
    requests=[]
    for name,start,goal in pairs:
        for direction,a,b in [('forward',start,goal),('reverse',goal,start)]:
            requests.append(dict(id=f'{name}_{direction}',start=a,goal=b,
                                 start_yaw=math.atan2(b[1]-a[1],b[0]-a[0]),expected=case['expected']))
    if case['id']=='cross_55':
        for offset in (.05,.1):
            for direction,a,b in [('forward',pairs[0][1],pairs[0][2]),('reverse',pairs[0][2],pairs[0][1])]:
                requests.append(dict(id=f'raised_{round(offset*100):02}_{direction}',
                    start=[a[0],a[1],a[2]+offset],goal=[b[0],b[1],b[2]+offset],
                    start_yaw=math.atan2(b[1]-a[1],b[0]-a[0]),expected='route'))
    result={k:v for k,v in case.items() if k!='fn'}
    result.update(resolution=R,map_path=str(map_path),x=x.tolist(),y=y.tolist(),
                  heights=[[float(v) if keep else None for v,keep in zip(row,valid)] for row,valid in zip(heights,mask)],
                  ideal_heights=ideal.tolist(),extras=extra,requests=requests,
                  max_surface_angle_deg=float(np.degrees(np.arctan(np.hypot(dx,dy))).max()),
                  max_voxel_rise_m=float(max(abs(np.diff(heights,axis=0)).max(),abs(np.diff(heights,axis=1)).max())),
                  occupied_cells=int(made.stdout.strip().splitlines()[-1]))
    return result


def run(scene,request,timeout):
    folder=OUT/'runs'/scene['id']/request['id'];folder.mkdir(parents=True,exist_ok=True)
    payload=dict(map_path=scene['map_path'],start=request['start'],goal=request['goal'],
                 start_yaw=request['start_yaw'],options=OPTIONS)
    write(folder/'request.json',payload)
    began=time.perf_counter();geometry={};plan={}
    try:
        p=subprocess.run([str(BIN/'octoplanner3d_headless.exe')],input=json.dumps(payload),capture_output=True,text=True,timeout=timeout)
        (folder/'plan.log').write_text(p.stderr,encoding='utf-8')
        plan=json.loads(p.stdout);plan['returncode']=p.returncode;write(folder/'plan.json',plan)
        path=plan.get('path',[])
        if p.returncode not in (0,2):status='runner_failed'
        elif not plan.get('ok'):
            status='inconclusive' if 'iteration' in plan.get('error','') else 'no_route'
        else:
            route=folder/'route.xyz';route.write_text('\n'.join(' '.join(map(str,p)) for p in path))
            cmd=[str(BIN/'octoplanner3d_verify_path.exe'),'--map',scene['map_path'],'--path',str(route),'--planning-anchor',*map(str,request['start']),'--planning-anchor-yaw',str(request['start_yaw'])]
            for key,flag in FLAGS.items():cmd.extend(['--'+flag,str(plan['diagnostics']['constraints'][key]).lower()])
            q=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout)
            geometry=json.loads(q.stdout);geometry['returncode']=q.returncode;write(folder/'geometry.json',geometry)
            status='rust_pass' if (plan.get('reached_goal') and plan['diagnostics'].get('used_rust_graph') and
                not plan['diagnostics'].get('rust_fallback') and q.returncode==0 and all(geometry.get(k) is True for k in ('valid','route_valid','sweep_valid'))) else 'geometry_failed'
            if status=='rust_pass' and request['expected']=='blocked':status='unexpected_route'
    except subprocess.TimeoutExpired as error:
        status='timeout';write(folder/'timeout.json',dict(command=error.cmd,timeout_s=timeout))
    except (OSError,ValueError,KeyError) as error:
        status='runner_failed';plan['runner_error']=str(error)
    path=plan.get('path',[])
    length=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
    return dict(scene=scene['id'],request=request['id'],status=status,expected=request['expected'],
                elapsed_s=time.perf_counter()-began,length_m=length,
                detour_ratio=length/math.dist(request['start'],request['goal']) if path else None,
                result=plan,geometry=geometry)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',action='append')
    parser.add_argument('--timeout',type=float,default=45)
    parser.add_argument('--generate-only',action='store_true')
    args=parser.parse_args()
    selected=[c for c in designs() if not args.scene or c['id'] in args.scene]
    old=json.loads((OUT/'scenes.json').read_text())['scenes'] if args.scene and (OUT/'scenes.json').exists() else []
    scenes=[scene for scene in old if scene['id'] not in args.scene] if args.scene else []
    selected_scenes=[]
    for case in selected:
        scene=generate(case);scenes.append(scene);selected_scenes.append(scene);print('MAP',scene['id'],scene['occupied_cells'],flush=True)
    write(OUT/'scenes.json',dict(options=OPTIONS,scenes=scenes))
    if args.generate_only:return
    old=json.loads((OUT/'results.json').read_text()) if args.scene and (OUT/'results.json').exists() else []
    rows=[row for row in old if row['scene'] not in args.scene] if args.scene else []
    for scene in selected_scenes:
        for request in scene['requests']:
            print('RUN',scene['id'],request['id'],flush=True)
            row=run(scene,request,args.timeout);rows.append(row)
            write(OUT/'results.json',rows)
            print('DONE',scene['id'],request['id'],row['status'],round(row['elapsed_s'],2),flush=True)
    print('TOTAL',len(scenes),len(rows),flush=True)


if __name__=='__main__':main()
