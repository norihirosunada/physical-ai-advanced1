"""Perception checks on a saved image and independent fixture truth. No robot commands."""
import json,argparse
from pathlib import Path
import cv2
import numpy as np
from camera_observation import detect
p=argparse.ArgumentParser();p.add_argument('image');p.add_argument('fixture');p.add_argument('--output',required=True);args=p.parse_args()
rgb=cv2.cvtColor(cv2.imread(args.image),cv2.COLOR_BGR2RGB);observations=detect(rgb)
fixture=json.loads(Path(args.fixture).read_text());results=[]
for item in fixture['objects']:
 observed=observations.get(item['id']);assert observed and 'position' in observed,(item['id'],observations)
 error=float(np.linalg.norm(np.array(observed['position'][:2])-item['position'][:2]));assert error<.003,(item['id'],error)
 yaw_error=abs((observed['yaw']-item['yaw']+np.pi/2)%np.pi-np.pi/2) if item['id']!='eraser' else None
 if yaw_error is not None:assert yaw_error<.1,(item['id'],yaw_error)
 results.append({'id':item['id'],'xy_error':error,'yaw_error_rad':yaw_error})
# Entirely absent objects and large artificial occlusion must not yield a target.
assert not detect(np.zeros_like(rgb))
for name,observation in observations.items():
 masked=rgb.copy();u,v=map(int,observation['pixel_center']);masked[max(0,v-40):v+40,max(0,u-40):u+40]=0
 assert name not in detect(masked)
 partial=rgb.copy();partial[max(0,v-40):v+40,u:u+40]=0
 assert name not in detect(partial),('partial occlusion accepted',name)
Path(args.output).write_text(json.dumps({'status':'PASS','results':results,'absent_and_occluded_checks':'PASS'},indent=2))
print(json.dumps(results,indent=2))
