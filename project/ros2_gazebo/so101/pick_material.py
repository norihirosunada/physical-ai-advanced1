"""Single contact-only pick/place using the same backend as the Japanese ROS server."""
import argparse
import json
from pathlib import Path
from motion_backend import MotionBackend, Node, rclpy

p=argparse.ArgumentParser()
args=p.parse_args()
from controller_lease import acquire
lease=acquire()
rclpy.init();node=Node('so101_material_pick');backend=MotionBackend(node)
result={'status':'FAIL','method':'physical contact and friction, no weld/attachment/teleport'}
try:
 backend.ready();result.update(backend.transfer([.18,-.10]),status='PASS')
except Exception as exc:
 result.update(error=str(exc),trace=backend.trace)
 print('FAIL:',exc,flush=True)
finally:
 Path('/workspace/artifacts/so101_pick.json').write_text(json.dumps(result,indent=2))
 node.destroy_node();rclpy.shutdown()
if result['status']!='PASS':raise SystemExit(1)
