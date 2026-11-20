/* Offline number reader for reefer controller displays.
   The user taps a number on the photo; we find the digit group around the tap,
   then decode each digit by sampling the seven segment zones (works for
   7-segment LCDs and, approximately, for dot-matrix / LCD fonts). */
(function(global){
"use strict";

function grayFromImageData(id){
  const {width:w,height:h,data:d}=id;const g=new Uint8ClampedArray(w*h);
  for(let i=0,j=0;i<g.length;i++,j+=4)g[i]=(d[j]*299+d[j+1]*587+d[j+2]*114)/1000;
  return {g,w,h};
}

// adaptive threshold: dark pixel if darker than local mean by a margin
function binarize(G,winFrac,k){
  const {g,w,h}=G;const W=w+1;const I=new Float64Array(W*(h+1));
  for(let y=0;y<h;y++){let s=0;for(let x=0;x<w;x++){s+=g[y*w+x];I[(y+1)*W+x+1]=I[y*W+x+1]+s}}
  const r=Math.max(4,Math.round(Math.max(w,h)*winFrac));const m=new Uint8Array(w*h);
  for(let y=0;y<h;y++){const y0=Math.max(0,y-r),y1=Math.min(h,y+r+1);
    for(let x=0;x<w;x++){const x0=Math.max(0,x-r),x1=Math.min(w,x+r+1);
      const mean=(I[y1*W+x1]-I[y0*W+x1]-I[y1*W+x0]+I[y0*W+x0])/((y1-y0)*(x1-x0));
      m[y*w+x]=g[y*w+x]<mean*(1-k)-4?1:0}}
  return m;
}
function dilate(m,w,h,r){
  if(r<1)return m;const t=new Uint8Array(w*h),o=new Uint8Array(w*h);
  for(let y=0;y<h;y++){let last=-1e9;for(let x=0;x<w;x++){if(m[y*w+x])last=x;if(x-last<=r)t[y*w+x]=1}
    last=1e9;for(let x=w-1;x>=0;x--){if(m[y*w+x])last=x;if(last-x<=r)t[y*w+x]=1}}
  for(let x=0;x<w;x++){let last=-1e9;for(let y=0;y<h;y++){if(t[y*w+x])last=y;if(y-last<=r)o[y*w+x]=1}
    last=1e9;for(let y=h-1;y>=0;y--){if(t[y*w+x])last=y;if(last-y<=r)o[y*w+x]=1}}
  return o;
}
function components(m,w,h){
  const lab=new Int32Array(w*h);const comps=[];const st=new Int32Array(w*h);
  for(let i=0;i<w*h;i++){if(!m[i]||lab[i])continue;
    const id=comps.length+1;let sp=0;st[sp++]=i;lab[i]=id;let x0=1e9,y0=1e9,x1=-1,y1=-1,n=0;
    while(sp){const p=st[--sp];const x=p%w,y=(p-x)/w;n++;if(x<x0)x0=x;if(x>x1)x1=x;if(y<y0)y0=y;if(y>y1)y1=y;
      if(x>0&&m[p-1]&&!lab[p-1]){lab[p-1]=id;st[sp++]=p-1}
      if(x<w-1&&m[p+1]&&!lab[p+1]){lab[p+1]=id;st[sp++]=p+1}
      if(y>0&&m[p-w]&&!lab[p-w]){lab[p-w]=id;st[sp++]=p-w}
      if(y<h-1&&m[p+w]&&!lab[p+w]){lab[p+w]=id;st[sp++]=p+w}}
    comps.push({id,x0,y0,x1,y1,n,w:x1-x0+1,h:y1-y0+1});}
  return {lab,comps};
}

function cleanMask(raw,w,h){
  const S=Math.max(w,h);const m=raw.slice();
  const LH=Math.round(S*0.1),LV=Math.round(S*0.15);
  for(let y=0;y<h;y++){let x=0;while(x<w){if(!raw[y*w+x]){x++;continue}let j=x;while(j<w&&raw[y*w+j])j++;if(j-x>LH)for(let k=x;k<j;k++)m[y*w+k]=0;x=j}}
  for(let x=0;x<w;x++){let y=0;while(y<h){if(!raw[y*w+x]){y++;continue}let j=y;while(j<h&&raw[j*w+x])j++;if(j-y>LV)for(let k=y;k<j;k++)m[k*w+x]=0;y=j}}
  const {lab,comps}=components(m,w,h);const kill=new Uint8Array(comps.length+1);
  for(const c of comps){if(c.n<3||c.w>S*0.25||c.h>S*0.25)kill[c.id]=1;
    else if((c.w>S*0.08&&c.h<c.w*0.12)||(c.h>S*0.12&&c.w<c.h*0.06))kill[c.id]=1}
  for(let i=0;i<w*h;i++)if(m[i]&&kill[lab[i]])m[i]=0;return m;
}
function locate(G,tx,ty){
  const {w,h}=G;const S=Math.max(w,h);
  const raw=binarize(G,1/14,0.13);const m=cleanMask(raw,w,h);
  tx=Math.round(tx);ty=Math.round(ty);
  const r=Math.max(2,Math.round(S/220));const dm=dilate(m,w,h,r);const {comps}=components(dm,w,h);
  const dist=c=>Math.hypot(Math.max(c.x0-tx,0,tx-c.x1),Math.max(c.y0-ty,0,ty-c.y1));
  const cand=comps.filter(c=>c.h>=S*0.015&&c.h<=S*0.25&&c.w<=S*0.5).map(c=>({c,d:dist(c)})).filter(o=>o.d<S*0.05).sort((a,b)=>a.d-b.d);
  if(!cand.length)return null;
  const y0=cand[0].c.y0+r,y1=cand[0].c.y1-r;
  const BH=y1-y0+1;if(BH<S*0.01)return null;
  return {m,w,h,tx,band:[y0,y1],BH,S};
}
function glyphs(L,shear){
  const {m,w,tx,band:[y0,y1],BH,S}=L;const ym=(y0+y1)/2;
  const X0=Math.max(0,tx-Math.round(S*0.35)),X1=Math.min(w-1,tx+Math.round(S*0.35));
  const N=X1-X0+1;const q=new Int32Array(N),top=new Int32Array(N).fill(1e9),bot=new Int32Array(N).fill(-1);
  for(let y=y0;y<=y1;y++){const sh=shear*(ym-y);for(let x=X0;x<=X1;x++){if(!m[y*w+x])continue;const X=Math.round(x+sh)-X0;if(X<0||X>=N)continue;q[X]++;if(y<top[X])top[X]=y;if(y>bot[X])bot[X]=y}}
  const runs=[];let i=0;while(i<N){if(!q[i]){i++;continue}let j=i,t=1e9,b=-1,n=0;while(j<N&&q[j]){t=Math.min(t,top[j]);b=Math.max(b,bot[j]);n+=q[j];j++}runs.push({x0:i+X0,x1:j-1+X0,y0:t,y1:b,n});i=j}
  if(!runs.length)return null;
  if(L.all){const H0=Math.max(...runs.map(r=>r.y1-r.y0+1));runs.forEach(r=>r._all=1);return finish(runs,H0)}
  // start run: containing/nearest tap
  const tX=tx;let si=0,bd=1e9;runs.forEach((r,k)=>{const d=Math.max(r.x0-tX,0,tX-r.x1);if(d<bd){bd=d;si=k}});
  const big=r=>(r.y1-r.y0+1)>=BH*0.55;
  if(!big(runs[si])){let b2=1e9,k2=-1;runs.forEach((r,k)=>{if(!big(r))return;const d=Math.max(r.x0-tX,0,tX-r.x1);if(d<b2){b2=d;k2=k}});if(k2<0||b2>BH*1.2)return null;si=k2}
  const H=runs[si].y1-runs[si].y0+1;const sel=[runs[si]];
  for(let k=si-1;k>=0;k--){if(sel[0].x0-runs[k].x1>H*0.4)break;sel.unshift(runs[k])}
  for(let k=si+1;k<runs.length;k++){if(runs[k].x0-sel[sel.length-1].x1>H*0.4)break;sel.push(runs[k])}
  return finish(sel,H);
  function finish(sel,H){
  const out=[];
  for(const r of sel){const gh=r.y1-r.y0+1,gw=r.x1-r.x0+1;
    if(gh>=H*0.55&&gw>gh*0.95){const parts=Math.max(2,Math.round(gw/(gh*0.62)));let a=r.x0;
      for(let p=1;p<parts;p++){const target=r.x0+gw*p/parts;let best=Math.round(target),bv=1e9;for(let x=Math.round(target-gw*0.12);x<=Math.round(target+gw*0.12);x++){const v=q[x-X0];if(v<bv){bv=v;best=x}}out.push(sub(a,best-1));a=best+1}
      out.push(sub(a,r.x1));}
    else out.push(r);}
  function sub(a,b){let t=1e9,bb=-1;for(let x=a;x<=b;x++){if(q[x-X0]){t=Math.min(t,top[x-X0]);bb=Math.max(bb,bot[x-X0])}}return{x0:a,x1:b,y0:t,y1:bb}}
  return {list:out.filter(r=>r.y1>=r.y0),H,ym};}
}
const GW=10,GH=16;let TPL=null;
const SEGS=[[.15,0,.85,.1],[.85,.05,1,.5],[.85,.5,1,.95],[.15,.9,.85,1],[0,.5,.15,.95],[0,.05,.15,.5],[.15,.45,.85,.55]];
const SEGPAT={"0":"1111110","1":"0110000","2":"1101101","3":"1111001","4":"0110011","5":"1011011","6":"1011111","7":"1110000","8":"1111111","9":"1111011","F":"1000111","C":"1001110"};
function gridFromCanvas(ctx,W,H){const d=ctx.getImageData(0,0,W,H).data;let x0=W,y0=H,x1=-1,y1=-1;
  for(let y=0;y<H;y++)for(let x=0;x<W;x++)if(d[(y*W+x)*4+3]>100){if(x<x0)x0=x;if(x>x1)x1=x;if(y<y0)y0=y;if(y>y1)y1=y}
  if(x1<0)return null;const bw=x1-x0+1,bh=y1-y0+1;const g=new Float32Array(GW*GH);
  for(let j=0;j<GH;j++)for(let i=0;i<GW;i++){let on=0,t=0;const ya=Math.floor(y0+j*bh/GH),yb=Math.max(ya+1,Math.floor(y0+(j+1)*bh/GH)),xa=Math.floor(x0+i*bw/GW),xb=Math.max(xa+1,Math.floor(x0+(i+1)*bw/GW));
    for(let y=ya;y<yb;y++)for(let x=xa;x<xb;x++){t++;if(d[(y*W+x)*4+3]>100)on++}g[j*GW+i]=on/t}
  return {g:norm(g),ar:bw/bh};}
function norm(g){let m=0;for(const v of g)m+=v;m/=g.length;let n=0;const o=new Float32Array(g.length);for(let i=0;i<g.length;i++){o[i]=g[i]-m;n+=o[i]*o[i]}n=Math.sqrt(n)||1;for(let i=0;i<g.length;i++)o[i]/=n;return o}
function buildTemplates(){
  TPL=[];const W=90,H=120;const cv=document.createElement("canvas");cv.width=W;cv.height=H;const ctx=cv.getContext("2d",{willReadFrequently:true});
  const fonts=["400 96px sans-serif","700 96px sans-serif","400 96px monospace","700 96px monospace","400 96px serif","400 96px Arial, Helvetica","700 96px Arial, Helvetica","400 96px 'Courier New', Courier","400 96px Verdana, sans-serif","400 96px 'Arial Narrow', 'Helvetica Neue', sans-serif"];
  const chars="0123456789FC";
  for(const f of fonts)for(const ch of chars){ctx.clearRect(0,0,W,H);ctx.font=f;ctx.fillStyle="#000";ctx.textBaseline="alphabetic";ctx.fillText(ch,8,100);const t=gridFromCanvas(ctx,W,H);if(t)TPL.push({ch,g:t.g,ar:t.ar,seg:false})}
  for(const [ch,p] of Object.entries(SEGPAT))for(const th of [0.12,0.18]){ctx.clearRect(0,0,W,H);ctx.fillStyle="#000";
    SEGS.forEach((s,i)=>{if(p[i]!=="1")return;const x0=10+s[0]*60,x1=10+s[2]*60,y0=5+s[1]*110,y1=5+s[3]*110;const tx=th*60;
      if(s[3]-s[1]<0.2)ctx.fillRect(x0,(y0+y1)/2-tx/2,x1-x0,tx);else ctx.fillRect((x0+x1)/2-tx/2,y0,tx,y1-y0)});
    const t=gridFromCanvas(ctx,W,H);if(t)TPL.push({ch,g:t.g,ar:t.ar,seg:true})}
}
function decodeGlyph(L,r,shear,ym){
  if(!TPL)buildTemplates();
  const {m,w}=L;const bw=r.x1-r.x0+1,bh=r.y1-r.y0+1;const ar=bw/bh;
  if(ar<0.3)return{d:"1",score:null};
  const g=new Float32Array(GW*GH);
  for(let j=0;j<GH;j++)for(let i=0;i<GW;i++){let on=0,t=0;const ya=Math.floor(r.y0+j*bh/GH),yb=Math.max(ya+1,Math.floor(r.y0+(j+1)*bh/GH));
    for(let y=ya;y<yb;y++){const sh=shear*(ym-y);const xa=Math.floor(r.x0+i*bw/GW-sh),xb=Math.max(xa+1,Math.floor(r.x0+(i+1)*bw/GW-sh));for(let x=xa;x<xb;x++){if(x<0||x>=w)continue;t++;on+=m[y*w+x]}}
    g[j*GW+i]=t?on/t:0}
  const G=norm(g);let best=null;
  for(const T of TPL){let s=0;for(let i=0;i<G.length;i++)s+=G[i]*T.g[i];s-=Math.abs(Math.log((ar+0.05)/(T.ar+0.05)))*0.15;if(!best||s>best.score)best={d:T.ch,score:s}}
  return best;
}
function readGroup(L){
  let best=null;
  for(const shear of [0,0.06,0.12,0.18,0.24]){
    const G=glyphs(L,shear);if(!G)continue;
    const H=G.H;let parts=[],sc=0,nd=0,unit=null,dotAt=-1,minus=false;
    for(const r of G.list){const gh=r.y1-r.y0+1,gw=r.x1-r.x0+1;
      if(gh<H*0.35){ // small: dot, minus or degree sign
        const cy=(r.y0+r.y1)/2;
        if(cy>G.ym+H*0.25&&gw<H*0.35){dotAt=parts.length;continue}
        if(Math.abs(cy-G.ym)<H*0.2&&gw>gh*1.4&&!parts.length){minus=true;continue}
        continue}
      if(gh<H*0.55)continue;
      const g=decodeGlyph(L,r,shear,G.ym);
      if(g.d==="F"||g.d==="C"){if(parts.length){unit=g.d;break}continue}
      parts.push(g.d);if(g.score!=null){sc+=g.score;nd++}}
    if(!parts.length)continue;sc=nd?sc/nd:0.5;
    if(!best||sc>best.score)best={parts,score:sc,unit,dotAt,minus,shear};
  }
  if(!best)return null;
  let digits=best.parts.join("");let txt=digits;
  if(best.dotAt>0&&best.dotAt<digits.length)txt=digits.slice(0,best.dotAt)+"."+digits.slice(best.dotAt);
  else if(digits.length>=2)txt=digits.slice(0,-1)+"."+digits.slice(-1);
  if(best.minus)txt="-"+txt;
  return {text:txt,value:parseFloat(txt),score:best.score,unit:best.unit,digits:digits.length};
}

function otsu(g){const hist=new Float64Array(256);for(const v of g)hist[v]++;const N=g.length;let sum=0;for(let i=0;i<256;i++)sum+=i*hist[i];
  let sB=0,wB=0,best=0,th=128;for(let t=0;t<256;t++){wB+=hist[t];if(!wB)continue;const wF=N-wB;if(!wF)break;sB+=t*hist[t];const mB=sB/wB,mF=(sum-sB)/wF;const v=wB*wF*(mB-mF)*(mB-mF);if(v>best){best=v;th=t}}return th}
// Read the number inside a box the user drew (source coords).
function readBox(source,box,pad){
  pad=pad==null?0.12:pad;{const sw=source.naturalWidth||source.width,sh=source.naturalHeight||source.height;const bw0=box.x1-box.x0,bh0=box.y1-box.y0;box={x0:Math.max(0,box.x0-bw0*pad),x1:Math.min(sw,box.x1+bw0*pad),y0:Math.max(0,box.y0-bh0*pad*1.5),y1:Math.min(sh,box.y1+bh0*pad*1.5)}}
  const bw=box.x1-box.x0,bh=box.y1-box.y0;if(bw<8||bh<8)return null;
  const k=Math.min(4,Math.max(0.2,140/bh));const W=Math.max(8,Math.round(bw*k)),H=Math.max(8,Math.round(bh*k));
  const cv=document.createElement("canvas");cv.width=W;cv.height=H;const ctx=cv.getContext("2d",{willReadFrequently:true});
  ctx.drawImage(source,box.x0,box.y0,bw,bh,0,0,W,H);
  const G=grayFromImageData(ctx.getImageData(0,0,W,H));
  // local threshold handles glare gradients; global Otsu as a floor
  const loc=binarize(G,0.18,0.08);const th=otsu(G.g);const m=new Uint8Array(W*H);
  for(let i=0;i<W*H;i++)m[i]=loc[i]&&G.g[i]<th+25?1:0;
  // drop frame lines and specks; drop blobs touching the border that are long and thin
  const {lab,comps}=components(m,W,H);const kill=new Uint8Array(comps.length+1);
  for(const c of comps){const edge=c.x0===0||c.y0===0||c.x1===W-1||c.y1===H-1;
    if(c.n<Math.max(4,W*H*0.0004))kill[c.id]=1;
    else if((c.w>W*0.6&&c.h<H*0.15)||(c.h>H*0.8&&c.w<W*0.04&&edge))kill[c.id]=1;
    else if(edge&&(c.h<H*0.3)&&(c.y0===0||c.y1===H-1))kill[c.id]=1;
    else if(edge&&(c.w>W*0.5||c.h>H*0.85))kill[c.id]=1;
    else if(c.w>c.h*1.6&&c.w>W*0.25)kill[c.id]=1;}
  for(let i=0;i<W*H;i++)if(m[i]&&kill[lab[i]])m[i]=0;
  const live=comps.filter(c=>!kill[c.id]);
  // cluster components that stack vertically into one glyph (7-segment pieces, dot-matrix gaps)
  const cl=live.map(c=>({x0:c.x0,x1:c.x1,y0:c.y0,y1:c.y1}));
  let merged=true;
  while(merged){merged=false;
    for(let i=0;i<cl.length&&!merged;i++)for(let j=i+1;j<cl.length&&!merged;j++){const A=cl[i],B=cl[j];
      const ox=Math.min(A.x1,B.x1)-Math.max(A.x0,B.x0)+1;const mw=Math.min(A.x1-A.x0+1,B.x1-B.x0+1);
      const gy=Math.max(A.y0,B.y0)-Math.min(A.y1,B.y1);
      if(ox>=mw*0.3&&gy<Math.max(2,0.13*Math.max(A.y1-A.y0,B.y1-B.y0))){cl[i]={x0:Math.min(A.x0,B.x0),x1:Math.max(A.x1,B.x1),y0:Math.min(A.y0,B.y0),y1:Math.max(A.y1,B.y1)};cl.splice(j,1);merged=true}}}
  const hh=r=>r.y1-r.y0+1,ww=r=>r.x1-r.x0+1;
  const tall=cl.filter(r=>ww(r)<=hh(r)*1.6&&hh(r)>=H*0.2).sort((a,b)=>hh(b)-hh(a));
  if(!tall.length)return null;
  // digit height: the tallest group of similar-height glyphs on one line
  let bestLine=null;
  for(const ref of tall){const line=tall.filter(r=>hh(r)>=hh(ref)*0.7&&hh(r)<=hh(ref)*1.2&&Math.min(r.y1,ref.y1)-Math.max(r.y0,ref.y0)>=Math.min(hh(r),hh(ref))*0.6);
    const sc=line.length*hh(ref);if(!bestLine||sc>bestLine.sc)bestLine={ref,line,sc}}
  const Hd=Math.max(...bestLine.line.map(hh));const top=Math.min(...bestLine.line.map(r=>r.y0)),bot=Math.max(...bestLine.line.map(r=>r.y1));
  let list=bestLine.line.slice();
  // small marks on the line: decimal point near baseline, minus at mid-height
  for(const r of cl){if(list.includes(r))continue;const h2=hh(r),w2=ww(r),cy=(r.y0+r.y1)/2;
    if(h2<Hd*0.3&&w2<Hd*0.4&&cy>top+Hd*0.7&&r.y1<=bot+Hd*0.15)list.push(Object.assign({},r,{small:"."}));
    else if(h2<Hd*0.25&&w2>h2*1.5&&w2>Hd*0.25&&w2<Hd*0.9&&Math.abs(cy-(top+bot)/2)<Hd*0.2){const first=Math.min(...bestLine.line.map(q=>q.x0));if(r.x1<first&&first-r.x1<Hd*0.6)list.push(Object.assign({},r,{small:"-"}))}}
  list.sort((a,b)=>a.x0-b.x0);
  // drop glyphs cut by the box edge (narrow and touching left/right)
  const typW=Math.max(...bestLine.line.map(ww));
  list=list.filter(r=>r.small||!((r.x0<=1||r.x1>=W-2)&&ww(r)<typW*0.55));
  list=list.filter(r=>r.small||ww(r)>=hh(r)*0.3||(hh(r)>=Hd*0.85&&r.y0>0&&r.y1<H-1));
  // split touching digits
  const out=[];
  for(const r of list){if(!r.small&&ww(r)>hh(r)*0.95){const parts=Math.max(2,Math.round(ww(r)/(hh(r)*0.62)));const q=[];for(let x=r.x0;x<=r.x1;x++){let s2=0;for(let y=r.y0;y<=r.y1;y++)s2+=m[y*W+x];q.push(s2)}
      let a=r.x0;for(let p=1;p<parts;p++){const t=ww(r)*p/parts;let bx=Math.round(t),bv=1e9;for(let x=Math.round(t-ww(r)*0.12);x<=Math.round(t+ww(r)*0.12);x++){if(q[x]<bv){bv=q[x];bx=x}}out.push({x0:a,x1:r.x0+bx-1,y0:r.y0,y1:r.y1});a=r.x0+bx+1}
      out.push({x0:a,x1:r.x1,y0:r.y0,y1:r.y1})}else out.push(r)}
  const L={m,w:W,h:H};const ym=(top+bot)/2;
  let best=null;
  for(const shear of [0,0.08,0.16,0.24]){
    let parts=[],sc=0,nd=0,unit=null,dotAt=-1,minus=false;
    for(const r of out){
      if(r.small==="."){if(parts.length)dotAt=parts.length;continue}
      if(r.small==="-"){if(!parts.length)minus=true;continue}
      if(hh(r)<Hd*0.6)continue;
      const g=decodeGlyph(L,r,shear,ym);
      if(g.d==="F"||g.d==="C"){if(parts.length){unit=g.d;break}continue}
      parts.push(g.d);if(g.score!=null){sc+=g.score;nd++}}
    if(!parts.length)continue;sc=nd?sc/nd:0.5;
    if(!best||sc>best.score)best={parts,score:sc,unit,dotAt,minus}}
  if(!best)return null;
  const digits=best.parts.join("");let txt=digits;
  if(best.dotAt>0&&best.dotAt<digits.length)txt=digits.slice(0,best.dotAt)+"."+digits.slice(best.dotAt);
  else if(digits.length>=2)txt=digits.slice(0,-1)+"."+digits.slice(-1);
  if(best.minus)txt="-"+txt;
  return {text:txt,value:parseFloat(txt),score:best.score,unit:best.unit,digits:digits.length};
}
// Main entry: canvas/image source -> {value,text,score,box} in source coordinates
function readAt(source,sw,sh,tx,ty,workMax){
  workMax=workMax||1100;const k=Math.min(1,workMax/Math.max(sw,sh));
  const W=Math.round(sw*k),H=Math.round(sh*k);
  const cv=document.createElement("canvas");cv.width=W;cv.height=H;const ctx=cv.getContext("2d",{willReadFrequently:true});
  ctx.drawImage(source,0,0,W,H);
  const G=grayFromImageData(ctx.getImageData(0,0,W,H));
  const L=locate(G,tx*k,ty*k);if(!L)return null;
  return readGroup(L);
}
global.ReeferOCR={readAt,readBox,_locate:locate,_glyphs:glyphs,_gray:grayFromImageData};
})(window);
