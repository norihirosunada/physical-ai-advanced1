"""Synthetic character n-gram classifier with conservative attribute filtering."""
import numpy as np
from catalog import MATERIALS, LABELS, PALETTE
ALIASES=[m['aliases'] for m in MATERIALS]
TEMPLATES=['{}を持ってきて','{}をください','{}に替えて','{}も追加して','{}をお願い','{}が欲しい']
def grams(s):return list(s)+[s[i:i+2] for i in range(len(s)-1)]
class Selector:
    def __init__(self):
        texts=[t.format(a) for group in ALIASES for a in group for t in TEMPLATES]
        self.vocab={g:i for i,g in enumerate(sorted({g for s in texts for g in grams(s)}))}
        x=self.encode(texts); y=np.eye(len(LABELS))[np.repeat(np.arange(len(LABELS)),18)]
        self.w=np.zeros((x.shape[1],len(LABELS)))
        for _ in range(1200):
            scores=x@self.w;p=np.exp(scores-scores.max(axis=1,keepdims=True));p/=p.sum(axis=1,keepdims=True)
            self.w-=0.7*(x.T@(p-y)/len(x)+0.0001*self.w)
    def encode(self,texts):
        x=np.zeros((len(texts),len(self.vocab)))
        for row,text in enumerate(texts):
            for g in grams(text):
                if g in self.vocab:x[row,self.vocab[g]]+=1
        return x/np.maximum(np.linalg.norm(x,axis=1,keepdims=True),1e-8)
    def select(self,text):
        colors=[jp for _,jp,_ in PALETTE if jp in text]
        if any(t in text for t in ['紫','オレンジ','橙','白','ピンク','茶','水色','灰色']):
            return {'status':'clarify','candidates':[],'text':text,'reason':'unsupported color'}
        kinds=[]
        if 'クレヨン' in text:kinds.append('クレヨン')
        if '色鉛筆' in text:kinds.append('色鉛筆')
        if '鉛筆' in text.replace('色鉛筆','') or any(t in text for t in ['えんぴつ','エンピツ']):kinds.append('鉛筆')
        if any(t in text for t in ['消しゴム','けしごむ','ケシゴム']):kinds.append('消しゴム')
        if len(colors)>1 or len(kinds)>1 or (not colors and not kinds):
            return {'status':'clarify','candidates':[],'text':text,'reason':'specify one material'}
        candidates=[m['id'] for m in MATERIALS if (not colors or m['color']==colors[0]) and (not kinds or m['kind']==kinds[0])]
        if len(candidates)!=1:
            return {'status':'clarify','candidates':candidates,'text':text,'reason':'ambiguous or unavailable'}
        scores=(self.encode([text])@self.w)[0];p=np.exp(scores-scores.max());p/=p.sum()
        target=candidates[0];idx=LABELS.index(target)
        if int(p.argmax())!=idx or p[idx]<0.40:
            return {'status':'clarify','candidates':candidates,'text':text,'reason':'low confidence'}
        return {'status':'selected','target':target,'confidence':float(p[idx]),'text':text}
