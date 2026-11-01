"""URDF forward kinematics and bounded numerical IK (known targets)."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parent
NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
URDF=ET.parse(ROOT/'vendor/so101_new_calib.urdf').getroot()
JOINTS={j.get('name'):j for j in URDF.findall('joint')}

def matrix(xyz,rpy):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz;return t

def forward(q,link='gripper_frame_link'):
    transforms={'base_link':matrix([0,0,1.015],[0,0,0])}
    values=dict(zip(NAMES,q));values['gripper']=0
    for name in NAMES+['gripper_frame_joint','gripper']:
        j=JOINTS[name];o=j.find('origin')
        xyz=list(map(float,o.get('xyz','0 0 0').split()));rpy=list(map(float,o.get('rpy','0 0 0').split()))
        motion=np.eye(4)
        if j.get('type')!='fixed':motion[:3,:3]=Rotation.from_rotvec(np.array(list(map(float,j.find('axis').get('xyz').split())))*values[name]).as_matrix()
        transforms[j.find('child').get('link')]=transforms[j.find('parent').get('link')]@matrix(xyz,rpy)@motion
    t=transforms[link].copy()
    if link=='gripper_frame_link':t[:3,3]+=t[:3,:3]@np.array([-0.012,0,0])
    return t

LOW=np.array([float(JOINTS[n].find('limit').get('lower')) for n in NAMES])
HIGH=np.array([float(JOINTS[n].find('limit').get('upper')) for n in NAMES])
def inverse(position,seed=None,yaw=0.):
    target_x=np.array([-np.cos(yaw),-np.sin(yaw),0.])
    def residual(q):
        t=forward(q)
        return np.r_[10*(t[:3,3]-position),t[:3,2]-[0,0,-1], t[:3,0]-target_x]
    best=None
    seeds=[np.zeros(5),[0,-0.8,1,-0.8,0],[0,0.8,-1,0.8,0]] if seed is None else [seed,np.zeros(5)]
    for initial in seeds:
        fit=least_squares(residual,np.clip(initial,LOW+1e-5,HIGH-1e-5),bounds=(LOW+1e-5,HIGH-1e-5),max_nfev=300)
        if best is None or np.linalg.norm(fit.fun)<np.linalg.norm(best.fun):best=fit
    t=forward(best.x)
    error=float(np.linalg.norm(t[:3,3]-position))
    if error>0.003 or np.linalg.norm(t[:3,2]-[0,0,-1])>0.05 or np.linalg.norm(t[:3,0]-target_x)>0.05:raise ValueError(f'IK failed: {position}, err={error}, residual={best.fun}')
    return best.x

if __name__=='__main__':
    print('zero pose',forward(np.zeros(5)))
    for p in [[0.18,0,1.03],[0.18,0,1.10],[0.18,-0.1,1.03]]:
        try:
            q=inverse(p);print(p,q,forward(q)[:3,3])
        except ValueError as e:print(e)
