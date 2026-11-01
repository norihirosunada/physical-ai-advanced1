"""Execute fetch/return pairs using the vision server and save every status."""
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
import json,time,argparse
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--materials',nargs='+',default=['消しゴム','青いクレヨン','赤いクレヨン']);args=p.parse_args()
rclpy.init();node=Node('scattered_command_check');events=[]
node.create_subscription(String,'/art_materials/status',lambda msg:events.append(json.loads(msg.data)),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
pub=node.create_publisher(String,'/art_materials/command',10)
def wait(predicate,timeout=240):
 deadline=time.monotonic()+timeout
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if predicate():return
 raise TimeoutError('task status timeout')
results=[];status='FAIL'
try:
 wait(lambda:events and pub.get_subscription_count()>0)
 for i,text in enumerate([text for name in args.materials for text in [name+'を持ってきて',name+'を戻して']]):
  request={'id':f'scattered-{i}','text':text}
  pub.publish(String(data=json.dumps(request,ensure_ascii=False)))
  def terminal():return [e for e in events if (e.get('request') or {}).get('id')==request['id'] and e['phase'] not in ['observing','executing']]
  wait(terminal);result=terminal()[-1];results.append(result);print(json.dumps(result,ensure_ascii=False),flush=True)
  assert result['phase']=='done',result
 status='PASS';print('PASS: camera-driven fetch/return',flush=True)
finally:
 Path('/workspace/artifacts/scattered_commands.json').write_text(json.dumps({'status':status,'results':results,'events':events},ensure_ascii=False,indent=2))
 node.destroy_node();rclpy.shutdown()
