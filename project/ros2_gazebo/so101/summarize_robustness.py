"""Host-side summary: keep baseline trials, failed tuning, and guard checks separate."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent/'artifacts'
rows=[]
first=json.loads((ROOT/'scattered_robustness_trial1.json').read_text())
rows.extend(first['results'])
for seed in [31,73]:
 for material in ['eraser','red_crayon','blue_crayon']:
  path=ROOT/f'isolated_seed_{seed}_{material}.json'
  if path.exists():rows.extend(json.loads(path.read_text())['results'])
retry=ROOT/'seed73_startup_retry.json'
if retry.exists():rows.extend(json.loads(retry.read_text())['results'])
expected={f'seed_{seed}_{material}' for seed in [19,31,73] for material in ['eraser','red_crayon','blue_crayon']}
assert len({r['case']['name'] for r in rows})==len(rows)
successes=[r for r in rows if r['passed']]
guard_path=ROOT/'guards_final.json'
if not guard_path.exists():guard_path=ROOT/'isolated_guards.json'
guards=json.loads(guard_path.read_text())['results'] if guard_path.exists() else []
summary={'measurement_complete':{r['case']['name'] for r in rows}==expected,
 'scope':'Three predefined separated box layouts, one fetch per material; not a general success rate',
 'startup_only_timeouts':['isolated_seed_73_eraser.json','isolated_seed_73_red_crayon.json'],
 'physical_cases':len(rows),'physical_successes':len(successes),
 'failures':[{'case':r['case']['name'],'reason':r['result'].get('reason'),'source':r['request']['id']} for r in rows if not r['passed']],
 'control':'Original validated control restored: close=-0.15rad, duration=2s, no width bias',
 'guards':[{'case':r['case']['name'],'passed':r['passed'],'phase':r['result']['phase'],'nonpark_arm_messages':r.get('nonpark_arm_messages'),'gripper_messages':r.get('gripper_messages')} for r in guards],
 'failed_tuning':['gentler closing alone did not resolve seed19 blue failure','gentler closing plus width-based 2mm bias did not resolve it; both reverted'],
 'initial_evaluator_note':'trial1 before fields may be stale; physical outcomes come from task-server feedback and trace; later evaluator waits for fresh poses',
 'cases':[{'case':r['case']['name'],'passed':r['passed'],'phase':r['result']['phase'],'xy_error_mm':r['result'].get('xy_error',0)*1000 if r['result']['phase']=='done' else None,'max_other_displacement_m':r.get('max_other_displacement'),'wall_seconds':r['wall_seconds'],'sim_seconds':r['sim_seconds'],'artifact':r['result'].get('artifact'),'image':r.get('image')} for r in rows]}
(ROOT/'robustness_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
print(json.dumps({key:value for key,value in summary.items() if key not in ['cases','initial_evaluator_note']},ensure_ascii=False,indent=2))
