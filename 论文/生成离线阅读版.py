from pathlib import Path
import re, urllib.request, base64, concurrent.futures
p=Path(__file__).resolve().parent
paper=p/'气动舵面与CMG协同姿态控制方法及实验.md'
s=paper.read_text(encoding='utf-8')
s=s.replace('+r_\\nu\\frac', '\\\\&\\qquad+r_\\nu\\frac').replace('+r_{\\Delta\\tau}\\frac', '\\\\&\\qquad+r_{\\Delta\\tau}\\frac')
# 与正文保留的原式号对应。
needle='\\boldsymbol{\\tau}_{\\mathrm{CMG}}\n=\n-\\boldsymbol{A}(\\boldsymbol{\\gamma})\\boldsymbol{\\nu}\n}\n$$'
s=s.replace(needle,needle+'\n\n（式2）')
paper.write_text(s,encoding='utf-8')
css=(p/'katex.min.css').read_text(encoding='utf-8')
# 只保留现代浏览器使用的woff2字体，以便完全离线阅读。
css=re.sub(r'src:[^;]+;',lambda m:'src:'+next((v for v in m[0][4:-1].split(',') if '.woff2' in v),m[0][4:-1])+';',css)
urls=set(re.findall(r'url\(([^)]+)\)',css))
def get(url):
    b=urllib.request.urlopen('https://cdn.jsdelivr.net/npm/katex@0.16.22/dist/'+url,timeout=30).read()
    return url,'data:font/woff2;base64,'+base64.b64encode(b).decode()
with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
    for url,data in pool.map(get,urls): css=css.replace('url('+url+')','url('+data+')')
(p/'katex.offline.css').write_text(css,encoding='utf-8')
print('offline fonts',len(urls))
