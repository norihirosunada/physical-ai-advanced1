"""Independent scene-fixture evaluation of the running vision task server.
set_pose occurs ONLY between tasks with the arm parked; robot transport stays physical.
Each positive trial resets all three materials, then requests one image-driven fetch.
"""
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import argparse,json,math,subprocess,time,uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from make_scattered_world import layout
from camera_observation import CameraObservation
import cv2
from kinematics import NAMES

PARK=[1.3,0.,0.,0.,0.]
WORDS={'eraser':'消しゴム','red_crayon':'赤いクレヨン','blue_crayon':'青いクレヨン'}
class Evaluation(Node):
 def __init__(self):
  super().__init__('scattered_robustness_evaluation')
  self.events=[];self.joints={};self.poses={};self.arm_commands=[];self.grip_commands=[];self.sim_time=0.;self.clock_received=0.;self.pose_received={};self.active_baseline=None;self.max_displacements={};self.camera=CameraObservation(self)
  self.create_subscription(String,'/art_materials/status',lambda m:self.events.append(json.loads(m.data)),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(JointState,'/joint_states',lambda m:self.joints.update(zip(m.name,m.position)),10)
  self.create_subscription(Clock,'/clock',self.clock,10)
  for name in WORDS:self.create_subscription(Pose,f'/model/{name}/pose',lambda m,n=name:self.pose(n,m),10)
  self.create_subscription(JointTrajectory,'/so101_arm_controller/joint_trajectory',lambda m:self.arm_commands.append(m),10)
  self.create_subscription(JointTrajectory,'/so101_gripper_controller/joint_trajectory',lambda m:self.grip_commands.append(m),10)
  self.commands=self.create_publisher(String,'/art_materials/command',10)
 def pose(self,name,msg):
  self.poses[name]=[msg.position.x,msg.position.y,msg.position.z];self.pose_received[name]=time.monotonic()
  if self.active_baseline is not None:
   displacement=float(np.linalg.norm(np.array(self.poses[name][:2])-self.active_baseline[name][:2]))
   self.max_displacements[name]=max(self.max_displacements.get(name,0.),displacement)
 def clock(self,msg):
  self.sim_time=msg.clock.sec+msg.clock.nanosec/1e9;self.clock_received=time.monotonic()
 def wait(self,predicate,timeout=240):
  deadline=time.monotonic()+timeout
  while time.monotonic()<deadline:
   rclpy.spin_once(self,timeout_sec=.05)
   if predicate():return
  raise TimeoutError('evaluation feedback timeout')
 def settle(self,seconds):
  received_after=time.monotonic();self.wait(lambda:self.clock_received>received_after,15)
  start=self.sim_time;self.wait(lambda:self.sim_time-start>=seconds,45)
 def set_pose(self,name,position,yaw=0.):
  request=f'name: "{name}", position: {{x: {position[0]}, y: {position[1]}, z: {position[2]}}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
  for attempt in range(2):
   reply=subprocess.run(['ign','service','-s','/world/default/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','15000','--req',request],capture_output=True,text=True,timeout=20,check=True)
   if 'data: true' in reply.stdout:return
  raise RuntimeError(reply.stdout+reply.stderr)
 def configure(self,objects):
  assert self.events[-1]['phase'] in ['ready','done','noop','blocked','not_detected','unsupported','clarify','rejected'],self.events[-1]
  assert all(abs(self.joints[n]-v)<.02 for n,v in zip(NAMES,PARK)),self.joints
  # Stage apart first, to avoid collisions with the previous fixture during setup.
  for i,name in enumerate(WORDS):self.set_pose(name,[.4+.1*i,.45,1.013])
  for item in objects:self.set_pose(item['id'],item['position'],item.get('yaw',0.))
  configured=time.monotonic();self.settle(2.)
  self.wait(lambda:all(self.pose_received.get(name,0.)>configured for name in WORDS),15)
 def run(self,case):
  self.configure(case['objects']);before=dict(self.poses)
  observation=self.camera.observe(min_sim_time=self.sim_time)
  image_path=Path('/workspace/artifacts')/f'evaluation_scene_{uuid.uuid4().hex}.png'
  cv2.imwrite(str(image_path),cv2.cvtColor(self.camera.rgb,cv2.COLOR_RGB2BGR))
  request={'id':uuid.uuid4().hex,'text':WORDS[case['target']]+'を持ってきて'}
  arm_start=len(self.arm_commands);grip_start=len(self.grip_commands);event_start=len(self.events);start=time.monotonic();sim_start=self.sim_time
  self.active_baseline=before;self.max_displacements={name:0. for name in WORDS}
  self.commands.publish(String(data=json.dumps(request,ensure_ascii=False)))
  def terminal():return [e for e in self.events[event_start:] if (e.get('request') or {}).get('id')==request['id'] and e['phase'] not in ['observing','executing','busy']]
  self.wait(terminal);result=terminal()[-1];self.active_baseline=None
  evidence={'case':case,'request':request,'result':result,'before':before,'image':str(image_path),'observation':observation,'after':dict(self.poses),'max_other_displacement':{name:value for name,value in self.max_displacements.items() if name!=case['target']},'wall_seconds':time.monotonic()-start,'sim_seconds':self.sim_time-sim_start}
  if case['expected']!='done':
   arms=self.arm_commands[arm_start:];grips=self.grip_commands[grip_start:]
   nonpark=[m for m in arms if m.joint_names!=NAMES or len(m.points)!=1 or len(m.points[0].positions)!=5 or max(abs(a-b) for a,b in zip(m.points[0].positions,PARK))>.001]
   evidence.update(arm_messages=len(arms),nonpark_arm_messages=len(nonpark),gripper_messages=len(grips))
   assert not nonpark and not grips,evidence
   if result['phase']=='not_detected':assert result['location']=='unknown',evidence
   assert not any(e['phase']=='executing' and (e.get('request') or {}).get('id')==request['id'] for e in self.events[event_start:]),evidence
  evidence['passed']=result['phase']==case['expected'] and all(v<=.005 for v in evidence['max_other_displacement'].values())
  print(json.dumps({'case':case['name'],'phase':result['phase'],'expected':case['expected'],'passed':evidence['passed']},ensure_ascii=False),flush=True)
  return evidence

def guard_cases():
 def objects(eraser,red,blue):return [{'id':n,'position':p,'yaw':0.} for n,p in zip(WORDS,[eraser,red,blue])]
 return [
  {'name':'close_spacing','target':'eraser','expected':'blocked','objects':objects([.17,0,1.013],[.203,0,1.013],[.17,.09,1.013])},
  {'name':'target_outside_roi','target':'eraser','expected':'not_detected','objects':objects([.32,.20,1.013],[.235,.025,1.013],[.17,.085,1.013])},
  {'name':'destination_occupied','target':'eraser','expected':'blocked','objects':objects([.17,0,1.013],[.235,.025,1.013],[.18,-.10,1.013])},
  {'name':'target_occluded','target':'eraser','expected':'not_detected','objects':objects([.17,0,1.013],[.17,0,1.040],[.17,.085,1.013])}]

def main():
 p=argparse.ArgumentParser();p.add_argument('--seeds',nargs='*',type=int,default=[19,31,73]);p.add_argument('--materials',nargs='+',choices=list(WORDS),default=list(WORDS));p.add_argument('--guards',action='store_true');p.add_argument('--restore-seed',type=int);p.add_argument('--output',default='/workspace/artifacts/scattered_robustness.json');args=p.parse_args()
 cases=[{'name':f'seed_{seed}_{target}','seed':seed,'target':target,'expected':'done','objects':layout(seed)['objects']} for seed in args.seeds for target in args.materials]
 if args.guards:cases+=guard_cases()
 rclpy.init();node=Evaluation();report={'status':'RUNNING','results':[],'planned_cases':[c['name'] for c in cases],'fixture_note':'set_pose between idle tasks only; not robot transport'}
 def save():Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2))
 try:
  node.wait(lambda:node.events and len(node.poses)==3 and all(n in node.joints for n in NAMES) and node.commands.get_subscription_count()==1,180)
  for case in cases:
   result=node.run(case);report['results'].append(result);save()
   if result['result']['phase']=='error':break # Do not reset a faulted robot and conceal the failed trial.
  if args.restore_seed is not None and all(r['passed'] for r in report['results']):
   node.configure(layout(args.restore_seed)['objects']);report['restored_seed']=args.restore_seed
  report['events']=node.events
  report['status']='PASS' if len(report['results'])==len(cases) and all(r['passed'] for r in report['results']) else 'FAIL'
 except Exception as exc:report.update(status='FAIL',error=str(exc));raise
 finally:save();node.destroy_node();rclpy.shutdown()
 if report['status']!='PASS':raise SystemExit(1)
if __name__=='__main__':main()
