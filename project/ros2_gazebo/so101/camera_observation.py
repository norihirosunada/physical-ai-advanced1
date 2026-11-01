"""Color/geometry baseline for the calibrated overhead simulation camera.
No learned visual detector: the learned model remains the Japanese selector.
Ground-truth object topics are deliberately not read by this module.
"""
import json
import os
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp'
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

CAMERA_X=.195;CAMERA_Y=.015;CAMERA_Z=1.65;OBJECT_TOP=1.026

def detect(rgb):
 height,width=rgb.shape[:2];hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 masks={
  'red_crayon':cv2.inRange(hsv,np.array([0,100,70]),np.array([12,255,255])) | cv2.inRange(hsv,np.array([176,100,70]),np.array([179,255,255])),
  'blue_crayon':cv2.inRange(hsv,np.array([100,100,70]),np.array([135,255,255])),
  'eraser':cv2.inRange(hsv,np.array([150,50,70]),np.array([175,255,255]))}
 scale=2*(CAMERA_Z-OBJECT_TOP)*np.tan(.8/2)/width
 observations={}
 for name,mask in masks.items():
  count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask)
  candidates=[]
  for i in range(1,count):
   area=int(stats[i,cv2.CC_STAT_AREA])
   if area<120:continue
   u,v=centroids[i];points=np.column_stack(np.where(labels==i)[::-1]).astype(np.float32)
   covariance=np.cov(points.T);values,vectors=np.linalg.eigh(covariance);axis=vectors[:,np.argmax(values)]
   yaw=(np.arctan2(-axis[0],-axis[1])-np.pi/2+np.pi/2)%np.pi-np.pi/2
   if name=='eraser':
    box=cv2.boxPoints(cv2.minAreaRect(points));edge=box[1]-box[0]
    # A square has equivalent grasp axes modulo 90 degrees; PCA cannot resolve them.
    yaw=(np.arctan2(-edge[0],-edge[1])+np.pi/4)%(np.pi/2)-np.pi/4
   position=[float(CAMERA_X+(height/2-v)*scale),float(CAMERA_Y+(width/2-u)*scale),1.013]
   region='source' if .125<position[0]<.265 and -.055<position[1]<.135 else ('receiving' if .135<position[0]<.225 and -.14<position[1]<-.06 else None)
   if region is None:continue
   expected_area=(.022*.022 if name=='eraser' else .018*.042)/(scale*scale)
   if not .8<area/expected_area<1.4:continue
   candidates.append({'position':position,'yaw':float(yaw),'pixels':area,'pixel_center':[float(u),float(v)],'region':region,'method':'calibrated RGB color and PCA'})
  if len(candidates)==1:observations[name]=candidates[0]
  elif candidates:observations[name]={'status':'ambiguous','candidates':candidates}
 return observations

class CameraObservation:
 def __init__(self,node):
  self.node=node;self.rgb=None;self.received=0.;self.stamp=None
  self.subscription=node.create_subscription(Image,'/art_materials/camera',self.image,qos_profile_sensor_data)
 def image(self,msg):
  if msg.encoding not in ['rgb8','bgr8']:return
  rgb=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
  self.rgb=rgb.copy() if msg.encoding=='rgb8' else rgb[:,:,::-1].copy()
  self.received=time.monotonic();self.stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec}
 def observe(self,timeout=45,min_sim_time=None):
  started=time.monotonic()
  while time.monotonic()-started<timeout:
   rclpy.spin_once(self.node,timeout_sec=.1)
   if self.received>started and (min_sim_time is None or self.stamp['sec']+self.stamp['nanosec']/1e9>min_sim_time):return {'stamp':self.stamp,'objects':detect(self.rgb)}
  raise TimeoutError('no fresh camera image')

if __name__=='__main__':
 rclpy.init();node=Node('camera_observation_check');camera=CameraObservation(node)
 try:
  result=camera.observe();print(json.dumps(result,indent=2),flush=True)
  Path('/workspace/artifacts/scattered_camera.json').write_text(json.dumps(result,indent=2))
  cv2.imwrite('/workspace/artifacts/scattered_camera.png',cv2.cvtColor(camera.rgb,cv2.COLOR_RGB2BGR))
 finally:node.destroy_node();rclpy.shutdown()
