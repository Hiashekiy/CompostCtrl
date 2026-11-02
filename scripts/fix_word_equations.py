"""Pandoc-compatible TeX and editable OMML with same-line SEQ numbering."""
from pathlib import Path
import re,subprocess,copy,json
from docx import Document
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches,Pt

ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'论文'
src=folder/'气动舵面与CMG协同姿态控制方法及实验.md'
reference=folder/'气动舵面与CMG协同姿态控制方法及实验.docx'
out=folder/'气动舵面与CMG协同姿态控制方法及实验-公式修正版.docx'
qa=folder/'排版检查/Word公式修复';qa.mkdir(parents=True,exist_ok=True)
s=src.read_text(encoding='utf-8')
s=re.sub(r'\{\\rm\s+([^{}]+)\}',lambda m:r'{\mathrm{'+m[1]+'}}',s)
s=s.replace(r'\frac1M',r'\frac{1}{M}').replace(r'\mathcal W',r'\mathcal{W}')
src.write_text(s,encoding='utf-8')
raw=qa/'原生公式转换.docx'
cmd=[r'D:/Program Files/pandoc-3.8-windows-x86_64/pandoc-3.8/pandoc.exe',str(src),
 '--from=markdown+tex_math_dollars','--to=docx','--reference-doc='+str(reference),'-o',str(raw)]
r=subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8')
(qa/'pandoc转换日志.txt').write_text(r.stderr,encoding='utf-8')
assert r.returncode==0,r.stderr
assert 'Could not convert TeX math' not in r.stderr,r.stderr
doc=Document(raw)
sec=doc.sections[0]
# Original Pandoc reference may omit explicit paper dimensions. Match its usual A4 layout.
if sec.page_width is None:sec.page_width=Inches(8.2677)
if sec.page_height is None:sec.page_height=Inches(11.6929)
width=int(sec.page_width-sec.left_margin-sec.right_margin)
W='http://schemas.openxmlformats.org/wordprocessingml/2006/main'
M='http://schemas.openxmlformats.org/officeDocument/2006/math'
def text_run(text):
    r=OxmlElement('w:r');t=OxmlElement('w:t');t.text=text;r.append(t);return r
def field(instr,cached):
    f=OxmlElement('w:fldSimple');f.set(qn('w:instr'),instr);f.append(text_run(cached));return f
def paragraph(align):
    p=OxmlElement('w:p');pr=OxmlElement('w:pPr');j=OxmlElement('w:jc');j.set(qn('w:val'),align);pr.append(j)
    spacing=OxmlElement('w:spacing');spacing.set(qn('w:before'),'100');spacing.set(qn('w:after'),'100');pr.append(spacing)
    p.append(pr);return p
count=0
for p in list(doc._element.body):
    if p.tag!=qn('w:p'):continue
    match=re.fullmatch(r'（式(\d+)）',''.join(t.text or '' for t in p.iter(qn('w:t'))))
    if not match:continue
    n=int(match[1]);equation=p.getprevious()
    maths=equation.findall('.//'+qn('m:oMath'))
    assert maths,f'Equation {n} was not converted'
    table=OxmlElement('w:tbl');pr=OxmlElement('w:tblPr')
    tw=OxmlElement('w:tblW');tw.set(qn('w:w'),str(width//635));tw.set(qn('w:type'),'dxa');pr.append(tw)
    borders=OxmlElement('w:tblBorders')
    for side in ['top','left','bottom','right','insideH','insideV']:
        edge=OxmlElement('w:'+side);edge.set(qn('w:val'),'nil');borders.append(edge)
    pr.append(borders);table.append(pr)
    grid=OxmlElement('w:tblGrid');widths=[int(width*.10)//635,int(width*.80)//635,int(width*.10)//635]
    for w in widths:
        c=OxmlElement('w:gridCol');c.set(qn('w:w'),str(w));grid.append(c)
    table.append(grid);row=OxmlElement('w:tr');rp=OxmlElement('w:trPr');rp.append(OxmlElement('w:cantSplit'));row.append(rp)
    for i,w in enumerate(widths):
        cell=OxmlElement('w:tc');cp=OxmlElement('w:tcPr');cw=OxmlElement('w:tcW');cw.set(qn('w:w'),str(w));cw.set(qn('w:type'),'dxa');cp.append(cw)
        va=OxmlElement('w:vAlign');va.set(qn('w:val'),'center');cp.append(va);cell.append(cp)
        para=paragraph('center' if i==1 else 'right')
        if i==1:
            for math in maths:para.append(copy.deepcopy(math))
        elif i==2:
            para.append(text_run('('))
            bm=OxmlElement('w:bookmarkStart');bm.set(qn('w:id'),str(500+n));bm.set(qn('w:name'),'Equation'+str(n));para.append(bm)
            para.append(field(' SEQ Equation \\* ARABIC ',str(n)))
            bm=OxmlElement('w:bookmarkEnd');bm.set(qn('w:id'),str(500+n));para.append(bm)
            para.append(text_run(')'))
        cell.append(para);row.append(cell)
    table.append(row);equation.addprevious(table);equation.getparent().remove(equation);p.getparent().remove(p);count+=1
assert count==27,count
# Replace numbers in textual references with real Word REF fields.
refs=0
for node in list(doc._element.body.iter(qn('w:t'))):
    value=node.text or ''
    if not re.search(r'式（\d+）',value):continue
    run=node.getparent()
    if run.tag!=qn('w:r'):continue
    parent=run.getparent();idx=parent.index(run);parts=re.split(r'(式（\d+）)',value)
    for part in parts:
        m=re.fullmatch(r'式（(\d+)）',part)
        additions=[text_run('式（'),field(' REF Equation'+m[1]+' \\h ',m[1]),text_run('）')] if m else [text_run(part)]
        for e in additions:parent.insert(idx,e);idx+=1
        refs+=bool(m)
    parent.remove(run)
doc.save(out)
plain=''.join(doc._element.body.itertext())
assert '$$' not in plain
assert not re.search(r'\\(?:rm|frac|varepsilon|mathcal|dot|tau)',plain)
math_count=len(doc._element.body.findall('.//'+qn('m:oMath')))
assert math_count==91,math_count
(qa/'公式结构核查.json').write_text(json.dumps({'native_OMML':math_count,'display_equations':count,'cross_references':refs,'raw_latex':False,'pandoc_warnings':r.stderr,'output':str(out)},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'native_OMML':math_count,'numbered_equations':count,'references':refs},ensure_ascii=False))
