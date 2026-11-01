"""TEST FIXTURE ONLY: emulate a user placing the eraser at a new source-tray pose.
Never imported by the task server. This is an inter-task fixture reset, not robot transport.
Requires the robot to be idle and parked. Other materials remain untouched.
"""
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import json,math,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from kinematics import NAMES
rclpy.init();node=Node('emulate_user_return');joint={};statuses=[]
node.create_subscription(JointState,'/joint_states',lambda m:joint.update(zip(m.name,m.position)),10)
node.create_subscription(String,'/art_materials/status',lambda m:statuses.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
try:
 deadline=time.monotonic()+15
 while time.monotonic()<deadline and (not statuses or not all(n in joint for n in NAMES)):rclpy.spin_once(node,timeout_sec=.1)
 assert statuses and statuses[-1]['phase'] in ['done','ready','noop'],statuses
 assert all(abs(joint[n]-v)<.02 for n,v in zip(NAMES,[1.3,0,0,0,0])),joint
 yaw=-.35;position=[.19,-.035,1.013]
 request=f'name: "eraser", position: {{x: {position[0]}, y: {position[1]}, z: {position[2]}}}, orientation: {{z: {math.sin(yaw/2)}, w: {math.cos(yaw/2)}}}'
 result=subprocess.run(['ign','service','-s','/world/default/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],text=True,capture_output=True,check=True)
 if 'data: true' not in result.stdout:raise RuntimeError(result.stdout+result.stderr)
 event={'type':'user_return_fixture','position':position,'yaw':yaw,'robot_parked':True,'previous_status':statuses[-1],'service_result':result.stdout}
 Path('/workspace/artifacts/user_return_fixture.json').write_text(json.dumps(event,ensure_ascii=False,indent=2));print(json.dumps(event,ensure_ascii=False),flush=True)
finally:node.destroy_node();rclpy.shutdown()
