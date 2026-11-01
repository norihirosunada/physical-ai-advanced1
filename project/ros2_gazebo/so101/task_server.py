"""Serialized Japanese ROS commands; commit location only after physical verification."""
import json
import sys
from pathlib import Path
import numpy as np
from motion_backend import MotionBackend, rclpy, Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
# Reuse the project's trained character classifier and material catalog.
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'art_assistant'))
from intent import Selector

DESTINATIONS={'storage':[.18,0.],'tray':[.18,-.10]}
class TaskServer(Node):
 def __init__(self):
  super().__init__('art_material_task_server')
  self.selector=Selector();self.backend=MotionBackend(self)
  self.busy=True;self.pending=None;self.location='unknown';self.transfers=0
  self.status=self.create_publisher(String,'/art_materials/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(String,'/art_materials/command',self.command,10)
 def report(self,phase,request=None,**details):
  payload={'phase':phase,'request':request,'location':self.location,'transfers':self.transfers,**details}
  self.status.publish(String(data=json.dumps(payload,ensure_ascii=False)))
  print(json.dumps(payload,ensure_ascii=False),flush=True)
 def command(self,msg):
  try:
   request=json.loads(msg.data) if msg.data.startswith('{') else {'text':msg.data}
   if not isinstance(request,dict) or not isinstance(request.get('text'),str):raise ValueError('text must be a string')
  except (ValueError,TypeError):self.report('rejected',reason='invalid command');return
  if self.busy or self.pending is not None:self.report('busy',request);return
  self.pending=request
 def execute(self,request):
  text=request['text'];selection=self.selector.select(text)
  if any(word in text for word in ['やめ','キャンセル','ないで','不要']):
   self.report('rejected',request,reason='cancel or negative instruction');return
  if selection['status']!='selected':self.report('clarify',request,selection=selection);return
  if selection['target']!='eraser':self.report('unsupported',request,selection=selection);return
  returning=any(word in text for word in ['戻','返却','しまって'])
  fetching=any(word in text for word in ['持って','ください','欲しい','お願い'])
  if returning==fetching:self.report('clarify',request,reason='specify fetch or return');return
  destination='storage' if returning else 'tray'
  if self.location=='unknown':self.report('rejected',request,reason='object location unknown; inspect simulation');return
  measured=self.backend.object_pos
  if np.linalg.norm(measured[:2]-DESTINATIONS[self.location])>.025 or not 1.005<measured[2]<1.04:
   self.location='unknown';self.report('rejected',request,reason='object moved outside known location');return
  if self.location==destination:self.report('noop',request,reason='already at destination');return
  self.report('executing',request,selection=selection)
  try:
   result=self.backend.transfer(DESTINATIONS[destination])
   self.location=destination;self.transfers+=1
   Path('/workspace/artifacts/so101_last_command.json').write_text(json.dumps({'request':request,**result},ensure_ascii=False,indent=2))
   self.report('done',request,xy_error=result['xy_error'],object=self.backend.object_pos.tolist())
  except Exception as exc:
   self.location='unknown';self.report('error',request,reason=str(exc))

def main():
 from controller_lease import acquire
 lease=acquire()
 rclpy.init();node=TaskServer()
 try:
  node.backend.ready();node.backend.hold(.5)
  for name,xy in DESTINATIONS.items():
   if np.linalg.norm(node.backend.object_pos[:2]-xy)<.025 and 1.005<node.backend.object_pos[2]<1.04:node.location=name
  node.busy=False;node.report('ready')
  while rclpy.ok():
   rclpy.spin_once(node,timeout_sec=.1)
   if node.pending is not None:
    request=node.pending;node.pending=None;node.busy=True
    try:node.execute(request)
    finally:node.busy=False
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
