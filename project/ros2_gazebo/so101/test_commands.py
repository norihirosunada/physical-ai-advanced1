"""Integration check against a running Gazebo and task_server; performs two transfers."""
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
rclpy.init();node=Node('art_material_command_check');events=[]
node.create_subscription(String,'/art_materials/status',lambda msg:events.append(json.loads(msg.data)),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub=node.create_publisher(String,'/art_materials/command',10)
def wait(predicate,timeout=180):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1)
  if predicate():return
 raise TimeoutError('task status timeout')
results=[]
try:
 wait(lambda:events and pub.get_subscription_count()>0)
 initial=events[-1];home=initial['location'];assert home in ['tray','storage'],initial
 fetch='消しゴムを持ってきて';ret='消しゴムを戻して'
 same=fetch if home=='tray' else ret;other=ret if home=='tray' else fetch
 cases=[('赤いクレヨンを持ってきて','unsupported'),('消しゴムを持ってこないで','rejected'),('赤を持ってきて','clarify'),(same,'noop'),(other,'done'),(other,'noop'),(same,'done')]
 for i,(text,expected) in enumerate(cases):
  request={'id':str(i),'text':text};pub.publish(String(data=json.dumps(request,ensure_ascii=False)))
  terminal=lambda:[e for e in events if (e.get('request') or {}).get('id')==str(i) and e['phase']!='executing']
  wait(terminal)
  result=terminal()[-1];results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
  assert result['phase']==expected,result
 assert results[-1]['location']==home
 assert results[-1]['transfers']==initial['transfers']+2
 print('PASS: seven commands, two verified physical transfers',flush=True)
finally:
 Path('/workspace/artifacts/so101_commands.json').write_text(json.dumps({'results':results,'events':events},ensure_ascii=False,indent=2))
 node.destroy_node();rclpy.shutdown()
