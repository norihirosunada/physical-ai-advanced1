"""Single catalog shared by language, physics and UI."""
PALETTE = [
    ('red','赤',(0.9,0.1,0.15)), ('blue','青',(0.1,0.3,0.95)),
    ('yellow','黄',(0.95,0.8,0.1)), ('green','緑',(0.1,0.65,0.25)),
    ('black','黒',(0.12,0.12,0.15)),
]
MATERIALS=[]
for kind, japanese in [('crayon','クレヨン'),('colored_pencil','色鉛筆')]:
    for color, jp, rgb in PALETTE:
        adj = '黄色い' if jp=='黄' else jp+'い'
        MATERIALS.append({'id':f'{color}_{kind}','name':f'{adj}{japanese}',
                          'color':jp,'kind':japanese,'rgb':rgb,
                          'aliases':[f'{adj}{japanese}',f'{jp}の{japanese}',f'{jp}色の{japanese}']})
MATERIALS += [
    {'id':'pencil','name':'鉛筆','color':None,'kind':'鉛筆','rgb':(0.7,0.5,0.2),'aliases':['鉛筆','えんぴつ','エンピツ']},
    {'id':'eraser','name':'消しゴム','color':None,'kind':'消しゴム','rgb':(0.95,0.65,0.75),'aliases':['消しゴム','けしごむ','ケシゴム']},
]
LABELS=[m['id'] for m in MATERIALS]
DISPLAY={m['id']:m['name'] for m in MATERIALS}
