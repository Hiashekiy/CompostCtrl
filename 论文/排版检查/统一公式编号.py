from pathlib import Path
import re
p=Path(__file__).resolve().parents[1]/'气动舵面与CMG协同姿态控制方法及实验.md'
s=p.read_text(encoding='utf-8')
old_to_new={}
counter=0
def number(m):
    global counter
    counter+=1
    if m[2]: old_to_new[int(m[2])]=counter
    return m[1]+f'\n\n（式{counter}）'
s=re.sub(r'(\$\$[\s\S]*?\$\$)(?:\n\n（式(\d+)）)?',number,s)
s=re.sub(r'式（(\d+)）',lambda m:'式（'+str(old_to_new.get(int(m[1]),int(m[1])))+'）',s)
p.write_text(s,encoding='utf-8')
print('display equations',counter,'reference map',old_to_new)
