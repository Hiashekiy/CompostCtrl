const fs=require('fs'),path=require('path');
const deps='C:/Users/zz/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules';
const katex=require('./katex.min.js');
const {marked}=require(deps+'/marked');
const {chromium}=require(deps+'/playwright');
(async()=>{
const dir=__dirname, stem='气动舵面与CMG协同姿态控制方法及实验';
let md=fs.readFileSync(path.join(dir,stem+'.md'),'utf8');
const math=[];
function hold(tex,displayMode){
 const html=katex.renderToString(tex.trim(),{displayMode,throwOnError:true,strict:'error',output:'htmlAndMathml'});
 const id=math.length;math.push({tex,displayMode,html});return `MATHPLACEHOLDER${id}END`;
}
md=md.replace(/\$\$([\s\S]*?)\$\$/g,(_,tex)=>'\n\n'+hold(tex,true)+'\n\n');
md=md.replace(/\$([^$\n]+)\$/g,(_,tex)=>hold(tex,false));
let body=marked.parse(md);
body=body.replace(/<p>（式(\d+)）<\/p>/g,'<p style="text-align:right;margin-top:-20px;font-size:14px">（$1）</p>');
body=body.replace(/MATHPLACEHOLDER(\d+)END/g,(_,i)=>math[+i].html);
let images=0;
body=body.replace(/src="([^"]+\.png)"/g,(_,src)=>{
 const file=decodeURIComponent(src.replace(/&amp;/g,'&'));
 const bytes=fs.readFileSync(file); images++;return 'src="data:image/png;base64,'+bytes.toString('base64')+'"';
});
const css=fs.readFileSync(path.join(dir,'katex.offline.css'),'utf8');
const html=`<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>${stem}</title><style>${css}
body{margin:0;background:#f1f3f5;color:#17212b;font:17px/1.85 "Microsoft YaHei",sans-serif}main{max-width:1080px;margin:30px auto;background:white;padding:48px 64px}h1{font-size:29px;text-align:center}h2{font-size:25px;margin-top:2em}h3{font-size:21px;margin-top:1.7em}h4{font-size:18px}p{margin:1em 0;text-align:justify}table{border-collapse:collapse;width:100%;font-size:14px;line-height:1.65;margin:20px 0}td,th{padding:9px;border:1px solid #cbd3db}th{background:#eef2f6}img{display:block;max-width:100%;height:auto;margin:24px auto}a{color:#17559a} .katex-display{overflow-x:auto;overflow-y:hidden;padding:14px 4px;font-size:.94em} .katex{font-size:1.08em}@media(max-width:800px){main{margin:0;padding:24px}body{font-size:16px}table{font-size:12px}}@media print{main{padding:0;margin:0}body{background:white}h2,h3,h4{break-after:avoid}img{max-height:250mm;object-fit:contain;break-inside:avoid}}
</style><main>${body}</main></html>`;
const target=path.join(dir,stem+'.html');fs.writeFileSync(target,html);
const browser=await chromium.launch({headless:true,executablePath:'C:/Users/zz/AppData/Local/ms-playwright/chromium-1223/chrome-win64/chrome.exe'});
const page=await browser.newPage({viewport:{width:1440,height:1100}});
await page.goto('file:///'+target.replaceAll('\\','/'));await page.evaluate(()=>document.fonts.ready);
const audit=await page.evaluate(()=>({images:[...document.images].map(i=>({ok:i.complete&&i.naturalWidth>0,w:i.naturalWidth})),errors:document.querySelectorAll('.katex-error').length,wide:[...document.querySelectorAll('.katex-display')].map((e,i)=>({i,width:e.scrollWidth,available:e.clientWidth})).filter(e=>e.width>e.available+2)}));
const qa=path.join(dir,'排版检查');fs.mkdirSync(qa,{recursive:true});
await page.screenshot({path:path.join(qa,'首页.png')});
for(const [key,label] of [['1.5 有限时域','优化公式'],['1.4 飞行器','耦合建模'],['1.3 CMG配置','参数来源'],['3.1 不同姿态','实验结果'],['3.3 预测时域','预测机制']]){
 const h=page.getByRole('heading').filter({hasText:key}).first();await h.scrollIntoViewIfNeeded();await page.evaluate(el=>window.scrollTo(0,el.getBoundingClientRect().top+window.scrollY-25),await h.elementHandle());await page.screenshot({path:path.join(qa,label+'.png')});
}
fs.writeFileSync(path.join(qa,'渲染检查.json'),JSON.stringify({math:math.length,displayMath:math.filter(m=>m.displayMode).length,images,...audit},null,2));
await browser.close();console.log(JSON.stringify({math:math.length,images,...audit}));
})().catch(e=>{console.error(e);process.exit(1)});
