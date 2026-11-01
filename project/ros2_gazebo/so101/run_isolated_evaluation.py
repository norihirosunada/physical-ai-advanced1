"""Host-side Docker runner. A failed physical trial is saved before a fresh world.
No in-process fault reset, and no set_pose during robot transport.
"""
import json,subprocess,shlex
from pathlib import Path

CONTAINER='pai-ros2-gazebo'
ROOT=Path(__file__).resolve().parent.parent/'artifacts'
def docker(*args,**kwargs):return subprocess.run(['docker',*args],**kwargs)
def restart(tag):
 docker('restart',CONTAINER,check=True,stdout=subprocess.DEVNULL)
 docker('exec','-d','-u','ubuntu','-e','SO101_WORLD=/workspace/worlds/so101_scattered.sdf',CONTAINER,'bash','-c',f'bash /workspace/scripts/launch_so101.bash > /workspace/artifacts/isolated_launch_{tag}.log 2>&1',check=True)
 docker('exec','-d','-u','ubuntu',CONTAINER,'bash','-c',f'source /opt/ros/humble/setup.bash; OPENBLAS_NUM_THREADS=1 python3 /workspace/so101/scattered_task_server.py > /workspace/artifacts/isolated_server_{tag}.log 2>&1',check=True)

cases=[('guards',['--seeds','--guards'])]+[(f'seed_{seed}_{material}',['--seeds',str(seed),'--materials',material]) for seed in [31,73] for material in ['eraser','red_crayon','blue_crayon']]
manifest={'status':'RUNNING','groups':[],'reset_rule':'Failed trials remain recorded; next group starts in a fresh world.'}
needs_restart=False
try:
 for i,(name,arguments) in enumerate(cases):
  if needs_restart:restart(str(i))
  destination=f'/workspace/artifacts/isolated_{name}.json'
  command='source /opt/ros/humble/setup.bash; OPENBLAS_NUM_THREADS=1 python3 /workspace/so101/evaluate_scattered.py '+shlex.join([*arguments,'--restore-seed','7','--output',destination])
  outcome=docker('exec','-u','ubuntu',CONTAINER,'bash','-lc',command)
  docker('cp',f'{CONTAINER}:{destination}',str(ROOT/Path(destination).name),check=True)
  report=json.loads((ROOT/Path(destination).name).read_text())
  manifest['groups'].append({'name':name,'exit_code':outcome.returncode,'status':report['status'],'file':Path(destination).name,'world_restarted_before':needs_restart})
  needs_restart=outcome.returncode!=0
  (ROOT/'isolated_evaluation_manifest.json').write_text(json.dumps(manifest,indent=2))
 if needs_restart:restart('final')
 manifest['status']='COMPLETE' # Completion of measurement, not a claim all trials succeeded.
finally:(ROOT/'isolated_evaluation_manifest.json').write_text(json.dumps(manifest,indent=2))
