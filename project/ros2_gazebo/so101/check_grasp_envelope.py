"""Geometric diagnostic of distal proxies in the gripper frame; not a full collision planner."""
import json,itertools,math
from pathlib import Path

def corners(center,size):return [[center[i]+signs[i]*size[i]/2 for i in range(3)] for signs in itertools.product([-1,1],repeat=3)]
def finger(angle):
 out=[]
 for x,y,z in corners([-.0051,-.0707274,.0190182],[.014,.024,.014]):
  # Gripper joint rotates about Z, followed by its fixed +pi/2 X mounting.
  rx=x*math.cos(angle)-y*math.sin(angle);ry=x*math.sin(angle)+y*math.cos(angle)
  out.append([.0202+rx,.0188-z,-.0234+ry])
 return out
def envelope(opening):
 points=[p for i in range(101) for p in finger(-.15+(opening+.15)*i/100)]
 points+=corners([.0077,0.,-.0234],[.0454,.0396,.0248])+corners([-.0144,-.000218,-.0941274],[.013,.014,.024])
 radius=max(math.hypot(p[0]-.0041,p[1]+.000218121) for p in points)
 gap=min(p[0] for p in finger(opening))-(-.0144+.013/2)
 return {'opening_rad':opening,'minimum_tip_gap_m':gap,'swept_tip_xy_radius_m':radius,'fits_22mm_proxy':gap>=.026}

if __name__=='__main__':
 result=[envelope(a) for a in [.8,.2]]
 assert result[1]['minimum_tip_gap_m']>.026
 assert result[1]['swept_tip_xy_radius_m']+.023+.003<.062
 Path('/workspace/artifacts/grasp_envelope.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2))
