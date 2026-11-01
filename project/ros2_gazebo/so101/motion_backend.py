"""Contact-only pick/place; success requires Gazebo object pose feedback."""
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from kinematics import NAMES,forward,inverse
from scipy.spatial.transform import Rotation

class MotionBackend:
 def __init__(self,node,material="eraser"):
  self.node=node;self.latest={};self.object_pos=None;self.sim_time=0.;self.trace=[];self.object_yaw=0.;self.pose_received=0.;self.material=material;self.object_rpy=[0.,0.,0.];self.samples=[];self.stage='idle';self.targets={};self.last_sample=-1.;self.guard=None;self.context=None
  self.open_angle=float(os.environ.get('SO101_OPEN_ANGLE','0.2'))
  if not .15<=self.open_angle<=.8:raise ValueError('SO101_OPEN_ANGLE outside tested model range')
  self.node.create_subscription(JointState,'/joint_states',lambda msg:self.latest.update(zip(msg.name,msg.position)),10)
  self.node.create_subscription(Pose,f'/model/{material}/pose',self.pose,10)
  self.node.create_subscription(Clock,'/clock',self.clock,10)
  self.arm=self.node.create_publisher(JointTrajectory,'/so101_arm_controller/joint_trajectory',10)
  self.grip=self.node.create_publisher(JointTrajectory,'/so101_gripper_controller/joint_trajectory',10)
 def pose(self,msg):
  self.object_pos=np.array([msg.position.x,msg.position.y,msg.position.z])
  q=msg.orientation
  self.object_rpy=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler("xyz").tolist()
  self.object_yaw=self.object_rpy[2]
  self.pose_received=time.monotonic()
 def clock(self,msg):
  self.sim_time=msg.clock.sec+msg.clock.nanosec/1e9
 def wait(self,condition,seconds=60):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   rclpy.spin_once(self.node,timeout_sec=0.05)
   self.sample()
   if self.guard is not None:self.guard()
   if condition():return
  raise TimeoutError('ROS feedback timeout')
 def sample(self):
  if self.sim_time-self.last_sample<.05 or self.object_pos is None or not all(n in self.latest for n in NAMES+['gripper']):return
  self.last_sample=self.sim_time
  self.samples.append({'stage':self.stage,'sim_time':self.sim_time,'joints':dict(self.latest),'targets':dict(self.targets),'arm_final_target_error':max([abs(self.latest[n]-self.targets[n]) for n in NAMES if n in self.targets] or [0.]),'object':self.object_pos.tolist(),'object_rpy':self.object_rpy,'context':self.context() if self.context else {}})
 def stop(self):
  # Hold measured joints; do not open an unverified payload or auto-retry.
  self.guard=None;self.stage='stopped'
  for pub,names in [(self.arm,NAMES),(self.grip,['gripper'])]:
   if not all(n in self.latest for n in names):continue
   msg=JointTrajectory();msg.joint_names=names;point=JointTrajectoryPoint()
   point.positions=[float(self.latest[n]) for n in names];point.velocities=[0.]*len(names);point.time_from_start=Duration(nanosec=200000000)
   msg.points=[point];pub.publish(msg)
  if self.object_pos is not None and all(n in self.latest for n in NAMES+['gripper']):self.snapshot('failure_hold')
  return {'hold_sent':True,'stop_confirmed':False,'measured_joints':dict(self.latest)}
 def hold(self,seconds):
  start=self.sim_time;self.wait(lambda:self.sim_time-start>=seconds)
 def publish(self,pub,names,target,seconds=2,confirm=True):
  self.targets.update(zip(names,map(float,target)))
  m=JointTrajectory();m.joint_names=names;t=JointTrajectoryPoint();t.positions=list(map(float,target));t.velocities=[0.]*len(names);t.time_from_start=Duration(sec=seconds);m.points=[t];pub.publish(m)
  if confirm:self.wait(lambda:max(abs(self.latest[n]-v) for n,v in zip(names,target))<0.01)
  else:self.hold(seconds+0.5)
  self.hold(0.3)
 def snapshot(self,stage):
  q=[self.latest[n] for n in NAMES];t=forward(q)
  item={'stage':stage,'sim_time':self.sim_time,'joints':q,'gripper':self.latest['gripper'],'object':self.object_pos.tolist(),'tool':t[:3,3].tolist(),'object_yaw':self.object_yaw,'material':self.material,'object_rpy':self.object_rpy,'context':self.context() if self.context else {}}
  self.trace.append(item);print(json.dumps(item),flush=True)
 def move(self,target,stage,yaw=0.):
  self.stage=stage
  current=np.array([self.latest[n] for n in NAMES]);start=forward(current)[:3,3]
  msg=JointTrajectory();msg.joint_names=NAMES
  fractions=[1.] if stage=='approach' else np.linspace(0.1,1,10)
  for fraction in fractions:
   current=inverse(start+(np.array(target)-start)*fraction,current,yaw=yaw)
   point=JointTrajectoryPoint();point.positions=current.tolist()
   duration=float(4*fraction);point.time_from_start=Duration(sec=int(duration),nanosec=int((duration%1)*1e9))
   msg.points.append(point)
  msg.points[-1].velocities=[0.]*5;self.targets.update(zip(NAMES,current.tolist()));self.arm.publish(msg)
  self.wait(lambda:max(abs(self.latest[n]-v) for n,v in zip(NAMES,current))<0.01)
  self.hold(0.3);self.snapshot(stage)
 
 def ready(self):
  self.wait(lambda:self.object_pos is not None and all(n in self.latest for n in NAMES+['gripper']) and self.arm.get_subscription_count()>0 and self.grip.get_subscription_count()>0,90)
 def park(self):
  self.stage='park'
  self.publish(self.arm,NAMES,[1.3,0.,0.,0.,0.],seconds=4)
 def transfer(self,destination,source=None,yaw=None):
  self.trace=[];self.samples=[];self.last_sample=-1.;self.stage='initial';started=time.monotonic();self.wait(lambda:self.pose_received>started);self.hold(0.5);self.snapshot('initial')
  source=self.object_pos.copy() if source is None else np.array(source,dtype=float)
  yaw=self.object_yaw if yaw is None else yaw
  yaw=(yaw+np.pi/2)%np.pi-np.pi/2
  dest=np.array([*destination,1.018])
  for target in [[*source[:2],1.08],[*source[:2],1.018],dest,[*dest[:2],1.08]]:inverse(np.array(target),yaw=yaw)
  self.stage='opening'
  self.publish(self.grip,['gripper'],[self.open_angle]);self.move([*source[:2],1.08],'approach',yaw)
  self.move([*source[:2],1.018],'descend',yaw)
  self.stage='closing'
  self.publish(self.grip,['gripper'],[-0.15],confirm=False);self.hold(0.5);self.snapshot('close')
  self.move([*source[:2],1.08],'lift',yaw);self.hold(0.5);self.snapshot('lift_settled')
  if self.object_pos[2]<source[2]+0.035:raise RuntimeError('object did not lift by 35mm')
  self.move([*dest[:2],1.08],'transport',yaw)
  if self.object_pos[2]<source[2]+0.035:raise RuntimeError('object slipped during transport')
  self.move(dest,'lower',yaw);self.stage='releasing';self.publish(self.grip,['gripper'],[self.open_angle]);self.hold(0.5);self.snapshot('release')
  self.move([*dest[:2],1.08],'retreat',yaw);self.hold(1);self.snapshot('final')
  error=float(np.linalg.norm(self.object_pos[:2]-dest[:2]))
  if error>0.025 or not 1.005<self.object_pos[2]<1.04:raise RuntimeError('placement outside destination')
  return {'xy_error':error,'trace':self.trace,'samples':self.samples,'open_angle':self.open_angle}
