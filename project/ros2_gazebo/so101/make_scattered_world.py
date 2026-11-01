"""Reproducible non-overlapping tray layouts; poses are fixtures, not perception."""
import argparse
import json
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parent
MATERIALS={'eraser':{'size':[.022,.022,.026],'color':[.95,.45,.6,1]},
 'red_crayon':{'size':[.018,.042,.026],'color':[.9,.08,.08,1]},
 'blue_crayon':{'size':[.018,.042,.026],'color':[.08,.18,.95,1]}}

def layout(seed):
 rng=random.Random(seed)
 for attempt in range(100):
  items=[]
  for name,spec in MATERIALS.items():
   for _ in range(1000):
    x=rng.uniform(.155,.235);y=rng.uniform(-.025,.105)
    if all((x-i['position'][0])**2+(y-i['position'][1])**2>=.065**2 for i in items):break
   else:break
   items.append({'id':name,**spec,'position':[x,y,1.013],'yaw':rng.uniform(-.6,.6)})
  if len(items)==len(MATERIALS):break
 else:raise RuntimeError('could not sample separated layout')
 return {'seed':seed,'position_range':{'x':[.155,.235],'y':[-.025,.105]},
         'min_center_spacing':.065,'yaw_range_rad':[-.6,.6],
         'overlap':False,'shape_note':'box proxies, not realistic pencils or crayons','objects':items}

def build(seed,output,camera=False):
 fixture=layout(seed)
 tree=ET.parse(ROOT.parent/'worlds/so101_pick.sdf');world=tree.getroot().find('world')
 original=world.find("model[@name='eraser']");world.remove(original)
 for item in fixture['objects']:
  model=ET.fromstring(ET.tostring(original));model.set('name',item['id'])
  model.find('pose').text=' '.join(map(str,[*item['position'],0,0,item['yaw']]))
  for box in model.findall('.//geometry/box/size'):box.text=' '.join(map(str,item['size']))
  model.find('.//visual/material/diffuse').text=' '.join(map(str,item['color']))
  # Uniform box inertia, mass fixed at 12g for this control fixture.
  a,b,c=item['size'];mass=.012
  inertia=model.find('.//inertia')
  for key,value in [('ixx',mass*(b*b+c*c)/12),('iyy',mass*(a*a+c*c)/12),('izz',mass*(a*a+b*b)/12)]:inertia.find(key).text=str(value)
  world.append(model)
 marker=ET.SubElement(world,'model',name='source_tray_region')
 ET.SubElement(marker,'static').text='true'
 ET.SubElement(marker,'pose').text='.195 .04 1.0005 0 0 0'
 link=ET.SubElement(marker,'link',name='marking')
 visual=ET.SubElement(link,'visual',name='tray_region')
 ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text='.14 .19 .001'
 ET.SubElement(ET.SubElement(visual,'material'),'diffuse').text='.22 .25 .28 1'
 if camera:
  ET.SubElement(world,'plugin',filename='ignition-gazebo-sensors-system',name='ignition::gazebo::systems::Sensors').append(ET.fromstring('<render_engine>ogre2</render_engine>'))
  model=ET.SubElement(world,'model',name='overhead_camera');ET.SubElement(model,'static').text='true'
  ET.SubElement(model,'pose').text='.195 .015 1.65 0 1.5707963267948966 0'
  link=ET.SubElement(model,'link',name='camera_link')
  sensor=ET.SubElement(link,'sensor',name='rgb',type='camera')
  for key,value in [('always_on','true'),('update_rate','5'),('topic','/art_materials/camera')]:ET.SubElement(sensor,key).text=value
  sensor.append(ET.fromstring('<camera><horizontal_fov>0.8</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.05</near><far>3</far></clip></camera>'))
 ET.indent(tree,space='  ');Path(output).write_text(ET.tostring(tree.getroot(),encoding='unicode'))
 Path(str(output)+'.json').write_text(json.dumps(fixture,indent=2))
 return fixture

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=7);p.add_argument('--output',required=True);p.add_argument('--camera',action='store_true');args=p.parse_args()
 print(json.dumps(build(args.seed,args.output,args.camera),indent=2))
