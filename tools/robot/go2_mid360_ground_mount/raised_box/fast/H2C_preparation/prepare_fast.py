source_path='D:\\inovxio\\brain\\lingtu\\tools\\robot\\go2_mid360_ground_mount\\raised_box\\fast\\print'
part_names=['box.stl','lid.stl','left_pad.stl','right_pad.stl']
positions=[(95,120),(240,115),(110,235),(240,235)]
overrides={'layer_height': '0.2', 'wall_loops': '6', 'top_shell_layers': '8', 'bottom_shell_layers': '8', 'sparse_infill_density': '45%', 'enable_support': '1', 'support_type': 'tree(auto)', 'support_filament': '2', 'support_interface_filament': '2', 'support_top_z_distance': '0.2', 'support_bottom_z_distance': '0.2'}
from pathlib import Path
import zipfile,xml.etree.ElementTree as E,numpy as np,json,trimesh
b=Path(r'D:/inovxio/thirdparty/夹爪辅助设计_开源/打印准备');out=Path('D:\\inovxio\\brain\\lingtu\\tools\\robot\\go2_mid360_ground_mount\\raised_box\\fast\\H2C_preparation')
ns='http://schemas.microsoft.com/3dmanufacturing/core/2015/02';pn='http://schemas.microsoft.com/3dmanufacturing/production/2015/06';q=lambda s:'{'+ns+'}'+s;p='{'+pn+'}'
E.register_namespace('',ns);E.register_namespace('p',pn)
z=zipfile.ZipFile('D:\\inovxio\\brain\\lingtu\\tools\\robot\\go2_mid360_ground_mount\\raised_box\\H2C_preparation\\H2C_V7_fast_parameters_comparison_NOT_SENT.3mf')
root=E.Element(q('model'),{'unit':'millimeter','requiredextensions':'p'});E.SubElement(root,q('metadata'),name='Application').text='BambuStudio-02.07.01.62';E.SubElement(root,q('metadata'),name='BambuStudio:3mfVersion').text='1';res=E.SubElement(root,q('resources'));build=E.SubElement(root,q('build'));cfg=E.Element('config');plate=E.Element('plate')
for k,v in [('plater_id','1'),('plater_name','V7_FAST_blue_PLA_028_4walls_20pct'),('locked','false'),('filament_map_mode','Manual'),('filament_maps','2 1'),('filament_volume_maps','0 0')]:E.SubElement(plate,'metadata',key=k,value=v)
rels=E.Element('Relationships',xmlns='http://schemas.openxmlformats.org/package/2006/relationships');files={};report=[]
parts=[trimesh.load_mesh(Path(source_path)/f,force='mesh') for f in part_names]
for idx in range(len(parts)):
 i=idx+1;m=parts[idx].copy()
 m.vertices-=np.r_[m.bounds.mean(0)[:2],m.bounds[0,2]]
 x,y=positions[idx];oid=str(2*i);vid=str(2*i-1);name=Path(part_names[idx]).stem
 assert m.is_watertight and m.is_winding_consistent
 m.export(out/(name+'.STL'))
 geom=E.Element(q('model'),unit='millimeter');gr=E.SubElement(geom,q('resources'));go=E.SubElement(gr,q('object'),id=vid,type='model');mesh=E.SubElement(go,q('mesh'));vs=E.SubElement(mesh,q('vertices'));ts=E.SubElement(mesh,q('triangles'))
 for v in m.vertices:E.SubElement(vs,q('vertex'),x=f'{v[0]:.9g}',y=f'{v[1]:.9g}',z=f'{v[2]:.9g}')
 for f in m.faces:E.SubElement(ts,q('triangle'),v1=str(f[0]),v2=str(f[1]),v3=str(f[2]))
 path=f'3D/Objects/object_{i}.model';files[path]=E.tostring(geom,encoding='utf-8',xml_declaration=True)
 obj=E.SubElement(res,q('object'),id=oid,type='model');comps=E.SubElement(obj,q('components'));E.SubElement(comps,q('component'),{p+'path':'/'+path,'objectid':vid,'transform':'1 0 0 0 1 0 0 0 1 0 0 0'})
 E.SubElement(build,q('item'),objectid=oid,transform=f'1 0 0 0 1 0 0 0 1 {x} {y} 0',printable='1')
 co=E.SubElement(cfg,'object',id=oid);E.SubElement(co,'metadata',key='name',value=name);E.SubElement(co,'metadata',key='extruder',value='2');E.SubElement(co,'metadata',face_count=str(len(m.faces)));part=E.SubElement(co,'part',id=vid,subtype='normal_part');E.SubElement(part,'metadata',key='name',value=name);E.SubElement(part,'metadata',key='matrix',value='1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1')
 inst=E.SubElement(plate,'model_instance')
 for k,val in [('object_id',oid),('instance_id','0'),('identify_id',str(600+i))]:E.SubElement(inst,'metadata',key=k,value=val)
 E.SubElement(rels,'Relationship',Target='/'+path,Id=f'rel-{i}',Type='http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel')
 report.append({'name':name,'size_mm':m.extents.tolist(),'watertight':bool(m.is_watertight),'winding_consistent':bool(m.is_winding_consistent),'volume_mm3':float(m.volume),'center_xy':[x,y],'orientation':'source print orientation preserved; XY translation and bed drop only'})
cfg.append(plate);E.SubElement(cfg,'assemble')
settings=json.loads(z.read('Metadata/project_settings.config'));settings['filament_map']=['2','1'];settings['filament_map_mode']='Manual';settings.update(__import__('json').loads(Path('D:\\inovxio\\brain\\lingtu\\tools\\robot\\go2_mid360_ground_mount\\raised_box\\H2C_preparation\\fast_prototype_overrides.json').read_text()))
for path,el in [('3D/3dmodel.model',root),('3D/_rels/3dmodel.model.rels',rels),('Metadata/model_settings.config',cfg)]:files[path]=E.tostring(el,encoding='utf-8',xml_declaration=True)
files['Metadata/project_settings.config']=json.dumps(settings,ensure_ascii=False,indent=2).encode();files['_rels/.rels']=z.read('_rels/.rels');files['[Content_Types].xml']=z.read('[Content_Types].xml')
dest=out/'H2C_V7_FAST_blue_PLA_028_4walls_20pct_待切片.3mf'
with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED) as z:
 for path,data in files.items():z.writestr(path,data)
with zipfile.ZipFile(dest) as z:
 assert z.testzip() is None
 for path in z.namelist():
  if path.endswith(('.model','.xml','.rels')) or path=='Metadata/model_settings.config':E.fromstring(z.read(path))
(out/'V7_FAST_blue_PLA_028_4walls_20pct_geometry.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(dest);print(json.dumps(report,ensure_ascii=False))
