"""Camera observations generate targets; Gazebo poses monitor execution and evaluate results.
Simple color segmentation is a visual baseline, not a learned detector.
"""
import json
from pathlib import Path
import time
import cv2
import numpy as np
from motion_backend import MotionBackend,rclpy
from camera_observation import CameraObservation
from check_grasp_envelope import envelope
from task_server import TaskServer

SUPPORTED=['eraser','red_crayon','blue_crayon']
RETURN_SLOTS={'eraser':[.17,0.],'red_crayon':[.235,.025],'blue_crayon':[.17,.085]}
class ScatteredTaskServer(TaskServer):
 def __init__(self):
  super().__init__()
  self.backends={'eraser':self.backend,**{name:MotionBackend(self,name) for name in SUPPORTED[1:]}}
  self.camera=CameraObservation(self);self.observations={};self.fault=False
 def observe(self):
  self.backend.park()
  return self.camera.observe(min_sim_time=self.backend.sim_time)
 def execute(self,request):
  text=request['text'];selection=self.selector.select(text)
  if any(word in text for word in ['やめ','キャンセル','ないで','不要']):self.report('rejected',request,reason='cancel or negative instruction');return
  if selection['status']!='selected':self.report('clarify',request,selection=selection);return
  target=selection['target']
  if target not in SUPPORTED:self.report('unsupported',request,selection=selection);return
  returning=any(word in text for word in ['戻','返却','しまって'])
  fetch_explicit=any(word in text for word in ['持って','欲しい'])
  if (returning and fetch_explicit) or any(word in text for word in ['替','換','交換','追加']):self.report('clarify',request,reason='currently supports fetch and return only');return
  if not returning and not (fetch_explicit or any(word in text for word in ['ください','お願い'])):self.report('clarify',request,reason='specify fetch or return');return
  if self.fault:self.report('rejected',request,reason='previous physical failure; inspect and restart');return
  self.report('observing',request,selection=selection)
  try:
   observation=self.observe();objects=observation['objects'];selected=objects.get(target)
   if selected is None or 'position' not in selected:
    self.location='unknown'
    self.report('not_detected',request,reason='target absent, occluded, ambiguous, or outside supported geometry');return
   self.location=selected['region']
   destination='source' if returning else 'receiving'
   if destination==self.location:self.report('noop',request,observation=selected);return
   if any(name not in objects or 'position' not in objects[name] for name in SUPPORTED):
    self.report('blocked',request,reason='not all three fixture objects are observable; inspect scene');return
   dest=RETURN_SLOTS[target] if returning else [.18,-.10]
   clearance=envelope(self.backends[target].open_angle)['swept_tip_xy_radius_m']+.023+.003
   for name,item in objects.items():
    if name==target or 'position' not in item:continue
    # Distal proxy sweep plus other-box radius and 3mm margin, not full link collision planning.
    if np.linalg.norm(np.array(item['position'][:2])-selected['position'][:2])<clearance or np.linalg.norm(np.array(item['position'][:2])-dest)<clearance:
     self.report('blocked',request,reason='insufficient separation or occupied destination',other=name);return
   backend=self.backends[target];backend.ready()
   # Evaluation truth is kept separate from the source sent to the controller.
   truth_before={name:b.object_pos.copy() for name,b in self.backends.items()}
   error=float(np.linalg.norm(np.array(selected['position'][:2])-truth_before[target][:2]))
   truth_yaw=backend.object_yaw
   maxima={name:0. for name in self.backends if name!=target}
   def monitoring():
    delta={name:float(np.linalg.norm(b.object_pos[:2]-truth_before[name][:2])) for name,b in self.backends.items() if name!=target}
    for name,value in delta.items():maxima[name]=max(maxima[name],value)
    return {'other_displacement':delta,'max_other_displacement':dict(maxima)}
   def guard():
    if any(value>.005 for value in monitoring()['other_displacement'].values()):raise RuntimeError('another material moved by more than 5mm during motion')
   for monitored in self.backends.values():monitored.context=monitoring;monitored.guard=guard
   self.report('executing',request,observation=selected)
   result=backend.transfer(dest,source=selected['position'],yaw=selected['yaw'])
   self.transfers+=1;self.location=destination
   after=self.observe()
   final=after['objects'].get(target)
   if final is None or final.get('region')!=destination:raise RuntimeError('target not re-observed at destination')
   disturbance={name:float(np.linalg.norm(b.object_pos[:2]-truth_before[name][:2])) for name,b in self.backends.items() if name!=target}
   if any(v>.005 for v in disturbance.values()):raise RuntimeError('another material moved by more than 5mm')
   artifact={'request':request,'observation':observation,'after':after,'perception_xy_error':error,'truth_yaw':truth_yaw,'max_other_displacement':dict(maxima),'other_displacement':disturbance,**result,'control_source':'RGB images generate targets; Gazebo poses monitor, stop and evaluate execution'}
   path=Path('/workspace/artifacts')/f'scattered_command_{time.time_ns()}.json';path.write_text(json.dumps(artifact,ensure_ascii=False,indent=2))
   cv2.imwrite(str(path.with_suffix('.png')),cv2.cvtColor(self.camera.rgb,cv2.COLOR_RGB2BGR))
   self.report('done',request,xy_error=result['xy_error'],perception_xy_error=error,other_displacement=disturbance,artifact=str(path))
  except Exception as exc:
   backend=self.backends[target]
   self.fault=True;self.location='unknown'
   hold={'hold_sent':False,'stop_confirmed':False}
   try:hold=backend.stop()
   except Exception as hold_exc:hold['error']=str(hold_exc)
   failure={'request':request,'status':'FAIL','error':str(exc),'trace':backend.trace,'samples':backend.samples,'open_angle':backend.open_angle,'hold':hold,'monitoring':backend.context() if backend.context else {}}
   Path('/workspace/artifacts/scattered_failure_'+str(time.time_ns())+'.json').write_text(json.dumps(failure,ensure_ascii=False,indent=2))
   self.report('error',request,reason=str(exc),monitoring=failure['monitoring'],hold=hold)
  finally:
   for monitored in self.backends.values():monitored.guard=None;monitored.context=None

def main():
 from controller_lease import acquire
 lease=acquire()
 rclpy.init();node=ScatteredTaskServer()
 try:
  for backend in node.backends.values():backend.ready()
  initial=node.observe();node.busy=False;node.report('ready',observation=initial)
  while rclpy.ok():
   rclpy.spin_once(node,timeout_sec=.1)
   if node.pending is not None:
    request=node.pending;node.pending=None;node.busy=True
    try:node.execute(request)
    finally:node.busy=False
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
