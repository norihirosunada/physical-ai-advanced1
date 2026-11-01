"""Record genuine ROS camera frames and task statuses, resampled at 2x simulation time."""
import json,time,uuid
from pathlib import Path
import cv2,numpy as np
from PIL import Image as PILImage,ImageDraw,ImageFont
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from evaluate_scattered import Evaluation,rclpy,layout

class Demo(Evaluation):
 def __init__(self):
  super().__init__();self.chapter='準備';self.caption='シミュレーション / 箱形代理物・非重複配置';self.frames=0;self.first_stamp=None;self.last_stamp=None;self.last_frame=None;self.enabled=False
  self.writer=cv2.VideoWriter('/workspace/artifacts/ros_reobservation_demo.mp4',cv2.VideoWriter_fourcc(*'mp4v'),10,(640,576))
  if not self.writer.isOpened():raise RuntimeError('video writer unavailable')
  self.fonts={size:ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',size) for size in range(11,18)}
  self.create_subscription(Image,'/art_materials/camera',self.frame,qos_profile_sensor_data)
 def frame(self,msg):
  if not self.enabled or msg.encoding not in ['rgb8','bgr8']:return
  stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
  if self.first_stamp is None:self.first_stamp=stamp
  rgb=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
  if msg.encoding=='bgr8':rgb=rgb[:,:,::-1]
  frame=PILImage.new('RGB',(640,576),(22,28,36));frame.paste(PILImage.fromarray(rgb),(0,0));draw=ImageDraw.Draw(frame)
  phase=self.events[-1]['phase'] if self.events else 'ready'
  def text(value,y,color):
   size=17
   while size>11 and draw.textbbox((0,0),value,font=self.fonts[size])[2]>616:size-=1
   draw.text((12,y),value,font=self.fonts[size],fill=color)
  text(self.chapter,482,'white')
  text(self.caption,507,(215,226,239))
  text(f'2倍速（シミュレーション時刻基準） | ROS: {phase}',545,(140,210,175))
  bgr=cv2.cvtColor(np.array(frame),cv2.COLOR_RGB2BGR)
  target=round((stamp-self.first_stamp)*5)
  while self.frames<target:
   self.writer.write(self.last_frame if self.last_frame is not None else bgr);self.frames+=1
  self.last_frame=bgr;self.last_stamp=stamp
 def send(self,text):
  request={'id':uuid.uuid4().hex,'text':text};start=len(self.events)
  self.chapter='日本語指示：'+text;self.caption='画像から位置・向きを再観察して搬送'
  self.commands.publish(String(data=json.dumps(request,ensure_ascii=False)))
  def done():return [e for e in self.events[start:] if (e.get('request') or {}).get('id')==request['id'] and e['phase'] not in ['observing','executing','busy']]
  self.wait(done);result=done()[-1]
  if result['phase']!='done':raise RuntimeError(str(result))
  return result

def main():
 rclpy.init();node=Demo();results=[];status='FAIL'
 try:
  node.wait(lambda:node.events and len(node.poses)==3 and all(n in node.joints for n in ['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']) and node.commands.get_subscription_count()==1,180)
  node.configure(layout(7)['objects']);node.enabled=True
  node.chapter='自由に使って戻せる画材トレイ';node.caption='日本語選択AI＋再観察＋SO-101 / 実機・利用者評価は未実施';node.settle(12)
  results.append(node.send('消しゴムを持ってきて'))
  results.append(node.send('消しゴムを戻して'))
  fixture=layout(7)['objects']
  fixture[0]={**fixture[0],'position':[.19,-.035,1.013],'yaw':-.35}
  node.chapter='利用者が別位置・別向きへ戻した後を再現';node.caption='評価用の配置変更 / ロボット停止・退避中のみ実施'
  node.configure(fixture);node.settle(8)
  results.append(node.send('消しゴムを持ってきて'))
  node.chapter='再観察して、新しい位置から取得できた';node.caption='固定色・既知高さ / Gazebo正解位置で監視・成否評価';node.settle(12)
  node.chapter='限定条件で検証 / 失敗も記録';node.caption='密接・未検出・受け取り先占有・遮蔽では取得を拒否';node.settle(10)
  status='PASS'
 finally:
  node.enabled=False;node.writer.release()
  report={'status':status,'results':results,'video_fps':10,'camera_simulation_fps':5,'speed':2,'frames':node.frames,'duration_seconds':node.frames/10,'sim_start':node.first_stamp,'sim_end':node.last_stamp,'resampling':'timestamp-based, hold previous frame for missing samples','scope':'actual camera and ROS statuses; user-return fixture change is explicitly captioned'}
  Path('/workspace/artifacts/ros_reobservation_demo.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
  node.destroy_node();rclpy.shutdown()
 if status!='PASS':raise SystemExit(1)
if __name__=='__main__':main()
