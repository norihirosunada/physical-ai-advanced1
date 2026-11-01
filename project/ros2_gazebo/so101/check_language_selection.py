"""Unseen-phrase check of material selection only, not full task grammar."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'art_assistant'))
from intent import Selector,TEMPLATES,ALIASES
cases=[
 ('青いクレヨンを手元まで持ってきて','blue_crayon'),
 ('赤色のクレヨンをいま使いたい','red_crayon'),
 ('消しゴムをこちらにお願いします','eraser'),
 ('青のクレヨンが必要です','blue_crayon'),
 ('けしごむを手元に置いてください','eraser'),
 ('赤のクレヨンを選びたい','red_crayon'),
 ('赤を手元に',None),('クレヨンをください',None),
 ('紫のクレヨンを持ってきて',None),('消しゴムと青いクレヨンを持ってきて',None)]
training={t.format(a) for group in ALIASES for a in group for t in TEMPLATES}
model=Selector();rows=[]
for text,target in cases:
 assert text not in training,text
 result=model.select(text)
 passed=result.get('target')==target and result['status']==('selected' if target else 'clarify')
 rows.append({'text':text,'expected':target,'result':result,'passed':passed})
report={'training_examples':len(training),'classes':12,'training_steps':1200,'learning_rate':.7,'l2':.0001,'scope':'material selection only; some phrasing may still need operation clarification','results':rows,'passed':sum(r['passed'] for r in rows)}
Path('/workspace/artifacts/language_unseen_phrases.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
if not all(r['passed'] for r in rows):raise SystemExit(1)
