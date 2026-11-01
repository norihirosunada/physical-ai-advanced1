"""Verify SO-101 arm and gripper against actual Gazebo JointState feedback."""
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration

NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
rclpy.init();node=Node('so101_motion_check');latest={}
node.create_subscription(JointState,'/joint_states',lambda msg:latest.update(zip(msg.name,msg.position)),10)
arm=node.create_publisher(JointTrajectory,'/so101_arm_controller/joint_trajectory',10)
grip=node.create_publisher(JointTrajectory,'/so101_gripper_controller/joint_trajectory',10)
def wait(condition,timeout=60):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=0.1)
        if condition():return True
    return False
def move(pub,names,target):
    msg=JointTrajectory();msg.joint_names=names
    point=JointTrajectoryPoint();point.positions=target;point.velocities=[0.0]*len(names);point.time_from_start=Duration(sec=2)
    msg.points=[point];pub.publish(msg)
    if not wait(lambda:max(abs(latest[n]-v) for n,v in zip(names,target))<0.015):raise RuntimeError('joint tracking timeout')
    return {n:latest[n] for n in names}
result={}
try:
    if not wait(lambda:all(n in latest for n in NAMES+['gripper']) and arm.get_subscription_count()>0 and grip.get_subscription_count()>0,90):raise RuntimeError('joint state or controllers missing')
    initial=[latest[n] for n in NAMES];initial_grip=latest['gripper']
    target=[0.2,0.1,-0.1,0.1,0.2]
    moved=move(arm,NAMES,target)
    opened=move(grip,['gripper'],[0.5]);closed=move(grip,['gripper'],[0.1])
    returned=move(arm,NAMES,initial);restored=move(grip,['gripper'],[initial_grip])
    result={'status':'PASS','robot':'SO-101 follower new calibration','simulator':'Gazebo Fortress',
            'initial':dict(zip(NAMES,initial)),'arm_target':target,'arm_measured':moved,
            'gripper_open':opened,'gripper_close':closed,'returned':returned,'gripper_restored':restored,'tolerance_rad':0.015}
    Path('/workspace/artifacts/so101_motion.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
finally:node.destroy_node();rclpy.shutdown()
