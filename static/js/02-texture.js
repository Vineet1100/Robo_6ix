/* =============================================================================
 * 02-texture.js — Aerial imagery of downtown Toronto
 * Draws the satellite-style city once at start-up: roads, lane markings, crosswalks, TTC tracks, trees, building roofs and shadows, and every landmark (TD Centre, Roy Thomson Hall, CN Tower…). TORONTO_BLOCKS chooses the style of each city block.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== aerial texture ==============================
const TEX=document.createElement('canvas');
const TW=(W+2*M)*TS;
let miniBase;

function drawCarTop(c,L,Wd,col,brake){
  rr(c,-L/2,-Wd/2,L,Wd,.45);c.fillStyle=col;c.fill();
  c.fillStyle='rgba(255,255,255,.10)';rr(c,-L/2+.15,-Wd/2+.12,L-.3,Wd*.3,.3);c.fill();
  c.fillStyle='#18212a';rr(c,L*.1,-Wd*.4,L*.17,Wd*.8,.18);c.fill();
  rr(c,-L*.35,-Wd*.37,L*.12,Wd*.74,.15);c.fill();
  c.fillStyle=shade(col.startsWith('#')?col:'#888888',-.08);c.fillRect(-L*.23,-Wd*.36,L*.33,Wd*.72);
  c.fillStyle='#fff6c9';c.fillRect(L/2-.12,-Wd/2+.15,.1,.35);c.fillRect(L/2-.12,Wd/2-.5,.1,.35);
  c.fillStyle=brake?'#ff2a3a':'#8c1b24';c.fillRect(-L/2+.02,-Wd/2+.15,.12,.38);c.fillRect(-L/2+.02,Wd/2-.53,.12,.38);
}
function drawTram(c,a){const L=a.L,w=a.Wd;rr(c,-L/2,-w/2,L,w,.9);c.fillStyle='#da291c';c.fill();
  c.fillStyle='#f2f2f0';c.fillRect(-L/2+1,-w*.32,L-2,w*.64);c.fillStyle='#d9d9d6';for(let k=-L/2+2.5;k<L/2-2;k+=3.2)c.fillRect(k,-w*.25,1.6,w*.5);
  c.fillStyle='#1d2328';c.fillRect(L/2-.9,-w*.4,.6,w*.8);c.fillRect(-L/2+.3,-w*.4,.6,w*.8);c.fillStyle='#333';c.fillRect(-.6,-.15,1.2,.3);
  c.strokeStyle='#8a1a12';c.lineWidth=.08;for(const k of [-L/6,L/6]){c.beginPath();c.moveTo(k,-w/2);c.lineTo(k,w/2);c.stroke();}
  c.fillStyle=a.acc<-1?'#ff2a3a':'#8c1b24';c.fillRect(-L/2,-w/2+.2,.1,.4);c.fillRect(-L/2,w/2-.6,.1,.4);}
function tree(g,x,y,r){
  g.fillStyle='rgba(8,12,6,.33)';g.beginPath();g.ellipse(x+r*.5,y+r*.5,r,r*.92,0,0,7);g.fill();
  const base=pick(['#2f4a22','#36522a','#2c4520','#3d5a2c','#41552a','#344d26'],SR);
  for(let k=0;k<3;k++){const ox=sr(-r*.3,r*.3),oy=sr(-r*.3,r*.3),r2=r*sr(.55,.82);
    const gr=g.createRadialGradient(x+ox-r2*.35,y+oy-r2*.35,r2*.1,x+ox,y+oy,r2);
    gr.addColorStop(0,shade(base,.28));gr.addColorStop(1,shade(base,-.3));
    g.fillStyle=gr;g.beginPath();g.arc(x+ox,y+oy,r2,0,7);g.fill();}
}
function roof(g,x,y,w,h,col){
  g.fillStyle=col;g.fillRect(x,y,w,h);
  g.fillStyle=shade(col,.07);g.fillRect(x+.6,y+.6,w-1.2,h-1.2);
  g.strokeStyle=shade(col,-.28);g.lineWidth=.35;g.strokeRect(x+.2,y+.2,w-.4,h-.4);
  const nn=Math.min(900,w*h*.5);
  for(let k=0;k<nn;k++){g.fillStyle=SR()<.5?'rgba(0,0,0,.07)':'rgba(255,255,255,.06)';g.fillRect(x+sr(.6,w-.8),y+sr(.6,h-.8),.25,.25);}
  const n=Math.floor(sr(1,4+w*h/160));
  for(let k=0;k<n;k++){const bw=sr(1,3.2),bh=sr(1,2.4);if(w-bw-2.4<0||h-bh-2.4<0)continue;
    const bx=x+sr(1.2,w-bw-1.2),by=y+sr(1.2,h-bh-1.2);
    g.fillStyle='rgba(0,0,0,.32)';g.fillRect(bx+.35,by+.35,bw,bh);
    g.fillStyle=pick(['#c9cbcc','#b3b6b8','#d8d8d2','#9ea3a6'],SR);g.fillRect(bx,by,bw,bh);
    g.strokeStyle='rgba(0,0,0,.25)';g.lineWidth=.08;g.strokeRect(bx,by,bw,bh);}
  if(SR()<.35&&w>12&&h>12){for(let k=0;k<4;k++){g.fillStyle='rgba(110,150,180,.55)';g.fillRect(x+w*.18+k*w*.17,y+h*.42,w*.08,h*.18);}}
}
function bDowntown(g,x,y,s){
  g.fillStyle='#9a968b';g.fillRect(x,y,s,s);
  for(let k=0;k<s;k+=1.5){g.fillStyle='rgba(0,0,0,.05)';g.fillRect(x+k,y,.06,s);g.fillRect(x,y+k,s,.06);}
  const L=SR();let parcels;
  if(L<.3)parcels=[[x+3,y+3,s-6,s-6]];
  else if(L<.65){const m=sr(.38,.62)*s;parcels=[[x+2,y+2,m-3,s-4],[x+m+1,y+2,s-m-3,s-4]];}
  else{const m=s/2;parcels=[[x+2,y+2,m-3,m-3],[x+m+1,y+2,m-3,m-3],[x+2,y+m+1,m-3,m-3],[x+m+1,y+m+1,m-3,m-3]];}
  const cols=['#b9b4a8','#8e9399','#a7a097','#c4c0b6','#7d8389','#9c8f80','#b0aaa0','#6f767d'];
  const bs=parcels.map(p=>{const i=sr(0,2.2);return[p[0]+i,p[1]+i,p[2]-2*i,p[3]-2*i,sr(10,48),pick(cols,SR)];});
  bs.forEach(b=>{const sh=b[4]*.13;g.fillStyle='rgba(12,14,10,.42)';g.fillRect(b[0]+sh,b[1]+sh,b[2],b[3]);});
  bs.forEach(b=>roof(g,b[0],b[1],b[2],b[3],b[5]));
}
function bResidential(g,x,y,s){
  g.fillStyle='#64793f';g.fillRect(x,y,s,s);
  const cols=3,rows=2,lw=s/cols,lh=s/rows;
  for(let c=0;c<cols;c++)for(let r=0;r<rows;r++){
    const lx=x+c*lw,ly=y+r*lh;
    g.fillStyle=pick(['#6a8044','#5d743b','#71864a','#62783e'],SR);g.fillRect(lx+.3,ly+.3,lw-.6,lh-.6);
    for(let k=0;k<lw;k+=1.2){g.fillStyle='rgba(255,255,255,.035)';g.fillRect(lx+k,ly,.6,lh);}
    g.strokeStyle='rgba(60,50,40,.5)';g.lineWidth=.12;g.strokeRect(lx+.3,ly+.3,lw-.6,lh-.6);
    const hw=sr(8,11),hh=sr(8,10.5),hx=lx+(lw-hw)/2+sr(-.8,.8),hy=r===0?ly+sr(3,5):ly+lh-hh-sr(3,5);
    g.fillStyle='#8f8b84';const dx=hx+hw-3.4;
    if(r===0)g.fillRect(dx,ly,3,hy-ly+1);else g.fillRect(dx,hy+hh-1,3,ly+lh-(hy+hh)+1);
    if(SR()<.35){const pw=sr(3.5,6),ph=sr(2.5,3.5),px=lx+sr(1.5,lw-pw-1.5),py=r===0?ly+lh-ph-sr(1.5,3):ly+sr(1.5,3);
      g.fillStyle='#e6e2d8';rr(g,px-.5,py-.5,pw+1,ph+1,.8);g.fill();g.fillStyle='#38afd3';rr(g,px,py,pw,ph,.6);g.fill();
      g.fillStyle='rgba(255,255,255,.28)';g.fillRect(px+.3,py+.3,pw*.5,.2);}
    g.fillStyle='rgba(12,14,10,.42)';g.fillRect(hx+1.1,hy+1.1,hw,hh);
    const rc=pick(['#7a4b3a','#5c5f66','#8a6a4f','#6b3f35','#4f5560','#7d756a','#3f4650'],SR),rH=SR()<.5;
    if(rH){g.fillStyle=shade(rc,.14);g.fillRect(hx,hy,hw,hh/2);g.fillStyle=shade(rc,-.2);g.fillRect(hx,hy+hh/2,hw,hh/2);}
    else{g.fillStyle=shade(rc,.14);g.fillRect(hx,hy,hw/2,hh);g.fillStyle=shade(rc,-.2);g.fillRect(hx+hw/2,hy,hw/2,hh);}
    g.strokeStyle='rgba(0,0,0,.13)';g.lineWidth=.06;
    for(let k=.5;k<(rH?hh:hw);k+=.5){g.beginPath();if(rH){g.moveTo(hx,hy+k);g.lineTo(hx+hw,hy+k);}else{g.moveTo(hx+k,hy);g.lineTo(hx+k,hy+hh);}g.stroke();}
    const nt=Math.floor(sr(1,3.6));for(let k=0;k<nt;k++)tree(g,lx+sr(2,lw-2),r===0?ly+lh-sr(2,6):ly+sr(2,6),sr(1.6,3));
  }
}
function bPark(g,x,y,s){
  g.fillStyle='#5b7f3a';g.fillRect(x,y,s,s);
  for(let k=0;k<s;k+=2.4){g.fillStyle='rgba(255,255,255,.04)';g.fillRect(x+k,y,1.2,s);}
  for(let k=0;k<140;k++){g.fillStyle=SR()<.5?'rgba(40,60,20,.13)':'rgba(150,150,80,.12)';g.beginPath();g.arc(x+sr(0,s),y+sr(0,s),sr(.5,2.6),0,7);g.fill();}
  g.strokeStyle='#c8b994';g.lineWidth=1.6;g.lineCap='round';
  g.beginPath();g.moveTo(x,y+s*.2);g.bezierCurveTo(x+s*.4,y+s*.1,x+s*.3,y+s*.9,x+s,y+s*.75);g.stroke();
  g.beginPath();g.moveTo(x+s*.15,y+s);g.bezierCurveTo(x+s*.3,y+s*.5,x+s*.7,y+s*.6,x+s*.8,y);g.stroke();
  const px=x+s*sr(.35,.65),py=y+s*sr(.35,.65),prx=sr(5,8),pry=sr(3.5,6);
  g.fillStyle='#b9a77d';g.beginPath();g.ellipse(px,py,prx+.9,pry+.9,.3,0,7);g.fill();
  g.fillStyle='#2b5966';g.beginPath();g.ellipse(px,py,prx,pry,.3,0,7);g.fill();
  g.fillStyle='rgba(120,180,190,.22)';g.beginPath();g.ellipse(px-1,py-1,prx*.6,pry*.5,.3,0,7);g.fill();
  const gx=x+3,gy=y+s-11;g.fillStyle='#c8ad73';rr(g,gx,gy,9,7,1.5);g.fill();
  ['#d64545','#3f7fd6','#e0c341'].forEach((c,k)=>{g.fillStyle='rgba(0,0,0,.3)';g.fillRect(gx+1.8+k*2.5,gy+1.8+(k%2)*2,1.6,2.2);g.fillStyle=c;g.fillRect(gx+1.5+k*2.5,gy+1.5+(k%2)*2,1.6,2.2);});
  for(let k=0;k<24;k++){const tx=x+sr(2,s-2),ty=y+sr(2,s-2);if(Math.hypot((tx-px)/(prx+2.5),(ty-py)/(pry+2.5))<1)continue;if(tx<gx+11&&ty>gy-2)continue;tree(g,tx,ty,sr(1.8,3.4));}
}
function bParking(g,x,y,s){
  g.fillStyle='#55585c';g.fillRect(x,y,s,s);
  for(let k=0;k<2500;k++){g.fillStyle=SR()<.5?'rgba(0,0,0,.12)':'rgba(255,255,255,.05)';g.fillRect(x+sr(0,s),y+sr(0,s),.2,.2);}
  for(const ry of [1.5,12.5,17.5,28.5,33.5]){
    for(let xx=x+3;xx<x+s-3-2.6;xx+=2.6){
      g.fillStyle='rgba(235,235,230,.7)';g.fillRect(xx,y+ry,.1,5);
      if(SR()<.62){g.save();g.translate(xx+1.3+.3,y+ry+2.5+.3);g.rotate(Math.PI/2);g.fillStyle='rgba(0,0,0,.35)';rr(g,-2.15,-.9,4.3,1.8,.4);g.fill();g.restore();
        g.save();g.translate(xx+1.3,y+ry+2.5);g.rotate(SR()<.5?Math.PI/2:-Math.PI/2);drawCarTop(g,4.3,1.8,pick(CARCOLS,SR),false);g.restore();}
    }
  }
  g.fillStyle='rgba(12,14,10,.4)';g.fillRect(x+s-9,y+s-8,7,6);roof(g,x+s-10,y+s-9,7,6,'#a59d90');
}
function bIndustrial(g,x,y,s){
  g.fillStyle='#8a857a';g.fillRect(x,y,s,s);
  for(let k=0;k<120;k++){g.fillStyle=SR()<.5?'rgba(90,80,60,.15)':'rgba(200,190,170,.12)';g.beginPath();g.arc(x+sr(0,s),y+sr(0,s),sr(.6,3),0,7);g.fill();}
  const ww=s*sr(.55,.72),wh=s*sr(.45,.58),wx=x+2,wy=y+2;
  g.fillStyle='rgba(12,14,10,.42)';g.fillRect(wx+2,wy+2,ww,wh);
  g.fillStyle='#c3c6c8';g.fillRect(wx,wy,ww,wh);
  for(let k=0;k<wh;k+=.9){g.fillStyle='rgba(0,0,0,.08)';g.fillRect(wx,wy+k,ww,.25);}
  g.fillStyle='rgba(0,0,0,.2)';g.fillRect(wx,wy+wh/2-.1,ww,.2);
  for(let k=0;k<7;k++){const cx=x+sr(3,s-10),cy=wy+wh+sr(3,s-wh-10);const cw=SR()<.5?6:12,ch=2.4,vert=SR()<.5;
    g.fillStyle='rgba(0,0,0,.35)';vert?g.fillRect(cx+.4,cy+.4,ch,cw):g.fillRect(cx+.4,cy+.4,cw,ch);
    g.fillStyle=pick(['#b8432f','#2f6db8','#cc8a2f','#3f8a4f','#8f8f8f'],SR);vert?g.fillRect(cx,cy,ch,cw):g.fillRect(cx,cy,cw,ch);}
}

// ---------- Downtown Toronto landmarks & block styles ----------
function shadowPoly(g,pts,ht){const sh=Math.min(ht*.13,16);g.fillStyle='rgba(12,14,10,.42)';g.beginPath();pts.forEach((p,i)=>i?g.lineTo(p[0]+sh,p[1]+sh):g.moveTo(p[0]+sh,p[1]+sh));g.closePath();g.fill();
  for(let i=0;i<pts.length;i++){const a=pts[i],b=pts[(i+1)%pts.length];g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.lineTo(b[0]+sh,b[1]+sh);g.lineTo(a[0]+sh,a[1]+sh);g.closePath();g.fill();}}
const rectPts=(x,y,w,h)=>[[x,y],[x+w,y],[x+w,y+h],[x,y+h]];
function poly(g,pts,fill){g.fillStyle=fill;g.beginPath();pts.forEach((p,i)=>i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1]));g.closePath();g.fill();}
function glassRoof(g,x,y,w,h,col,grid){roof(g,x,y,w,h,col);g.strokeStyle='rgba(255,255,255,.07)';g.lineWidth=.08;for(let k=grid;k<w;k+=grid){g.beginPath();g.moveTo(x+k,y+.6);g.lineTo(x+k,y+h-.6);g.stroke();}}
function plaza(g,x,y,s,col){g.fillStyle=col||'#a8a397';g.fillRect(x,y,s,s);g.fillStyle='rgba(0,0,0,.06)';for(let k=0;k<s;k+=1.5){g.fillRect(x+k,y,.05,s);g.fillRect(x,y+k,s,.05);}}
function bFinancial(g,x,y,s){plaza(g,x,y,s);const L=SR();let ps;
  if(L<.4)ps=[[x+3,y+3,s-6,s-6]];else if(L<.7){const m=sr(.4,.6)*s;ps=[[x+2,y+2,m-3,s-4],[x+m+1,y+2,s-m-3,s-4]];}
  else{const m=s/2;ps=[[x+2,y+2,m-3,m-3],[x+m+1,y+2,m-3,m-3],[x+2,y+m+1,m-3,m-3],[x+m+1,y+m+1,m-3,m-3]];}
  const bs=ps.map(p=>{const i=sr(.5,2.5);return[p[0]+i,p[1]+i,p[2]-2*i,p[3]-2*i,sr(60,180),pick(['#6f7a84','#59636d','#8a949c','#b8bcbf','#4f5a63','#9aa4ab'],SR)];});
  bs.forEach(b=>shadowPoly(g,rectPts(b[0],b[1],b[2],b[3]),b[4]));bs.forEach(b=>glassRoof(g,b[0],b[1],b[2],b[3],b[5],1.6));}
function bCondo(g,x,y,s){plaza(g,x,y,s,'#9d968a');
  const n=Math.floor(sr(3,5));for(let k=0;k<n;k++){const w=sr(10,18),h=sr(10,20),bx=x+sr(1,s-w-1),by=y+sr(1,s-h-1),ht=sr(12,30);shadowPoly(g,rectPts(bx,by,w,h),ht);roof(g,bx,by,w,h,pick(['#8c4a3a','#9a5b45','#7a4636','#a06a52','#6e3e31','#8b7d6b'],SR));}
  if(SR()<.8){const w=sr(12,17),h=sr(12,17),bx=x+sr(2,s-w-2),by=y+sr(2,s-h-2),ht=sr(90,150);shadowPoly(g,rectPts(bx,by,w,h),ht);glassRoof(g,bx,by,w,h,pick(['#6d8392','#8fa3b0','#7b8d99','#5f7380'],SR),1.4);}
  for(let k=0;k<3;k++)tree(g,x+sr(2,s-2),y+sr(2,s-2),sr(1.6,2.4));}
function bHeritage(g,x,y,s){plaza(g,x,y,s,'#8d877b');
  let cx=x;while(cx<x+s-3){const w=Math.min(sr(4.5,8),x+s-cx),d=sr(13,18),ht=sr(8,14);shadowPoly(g,rectPts(cx,y,w,d),ht);roof(g,cx,y,w-.15,d,pick(['#8c4a3a','#a0533f','#7d6a5a','#5d4a3e','#9a7b5b','#6b5a4c','#b07a52'],SR));cx+=w;}
  g.fillStyle='#7d786d';g.fillRect(x,y+19,s,3);
  const w2=s-4,h2=s-26;shadowPoly(g,rectPts(x+2,y+24,w2,h2),24);roof(g,x+2,y+24,w2,h2,pick(['#9a5b45','#8e9399','#a7a097'],SR));}
function bTD(g,x,y,s){plaza(g,x,y,s,'#b3ad9f');const t=[[x+2,y+3,16,30,220],[x+s-17,y+s-25,15,22,180],[x+s-15,y+2,12,12,40]];
  t.forEach(b=>shadowPoly(g,rectPts(b[0],b[1],b[2],b[3]),b[4]));
  t.forEach(b=>{g.fillStyle='#1b1e21';g.fillRect(b[0],b[1],b[2],b[3]);g.strokeStyle='rgba(255,255,255,.1)';g.lineWidth=.06;for(let k=1.5;k<b[2];k+=1.5){g.beginPath();g.moveTo(b[0]+k,b[1]);g.lineTo(b[0]+k,b[1]+b[3]);g.stroke();}g.strokeStyle='#2c3136';g.lineWidth=.4;g.strokeRect(b[0]+.3,b[1]+.3,b[2]-.6,b[3]-.6);});
  for(let k=0;k<4;k++)tree(g,x+sr(20,s-18),y+sr(5,18),1.6);}
function bRBP(g,x,y,s){plaza(g,x,y,s,'#aaa496');
  const t1=[[x+2,y+2],[x+30,y+2],[x+2,y+30]],t2=[[x+s-2,y+s-2],[x+s-30,y+s-2],[x+s-2,y+s-30]];
  shadowPoly(g,t1,180);shadowPoly(g,t2,130);
  poly(g,[[x+20,y+10],[x+s-10,y+s-20],[x+s-20,y+s-10],[x+10,y+20]],'#a8c4d4');
  for(const t of [t1,t2]){const gr=g.createLinearGradient(t[0][0],t[0][1],t[1][0],t[2][1]);gr.addColorStop(0,'#7a5a10');gr.addColorStop(.45,'#e6c65c');gr.addColorStop(.7,'#b8922a');gr.addColorStop(1,'#8d6c18');
    poly(g,t,gr);g.strokeStyle='rgba(60,40,0,.5)';g.lineWidth=.3;g.beginPath();t.forEach((p,i)=>i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1]));g.closePath();g.stroke();}}
function bFCP(g,x,y,s){plaza(g,x,y,s,'#a29d91');const podium=[x+1,y+1,s-2,s-2];roof(g,...podium,'#8a8f93');
  const c=[x+s/2,y+s/2],h=13,ch=3.5,oct=[[c[0]-h+ch,c[1]-h],[c[0]+h-ch,c[1]-h],[c[0]+h,c[1]-h+ch],[c[0]+h,c[1]+h-ch],[c[0]+h-ch,c[1]+h],[c[0]-h+ch,c[1]+h],[c[0]-h,c[1]+h-ch],[c[0]-h,c[1]-h+ch]];
  shadowPoly(g,oct,300);poly(g,oct,'#ecebe6');g.strokeStyle='rgba(0,0,0,.12)';g.lineWidth=.08;for(let k=-h+1;k<h;k+=1.2){g.beginPath();g.moveTo(c[0]+k,c[1]-h+.5);g.lineTo(c[0]+k,c[1]+h-.5);g.stroke();}
  g.fillStyle='#c9c9c4';g.fillRect(c[0]-4,c[1]-4,8,8);}
function bScotia(g,x,y,s){plaza(g,x,y,s,'#a59f93');const x0=x+8,y0=y+6,w=26,h=26,n=3;
  const p=[[x0+n,y0],[x0+w-n,y0],[x0+w-n,y0+n],[x0+w,y0+n],[x0+w,y0+h-n],[x0+w-n,y0+h-n],[x0+w-n,y0+h],[x0+n,y0+h],[x0+n,y0+h-n],[x0,y0+h-n],[x0,y0+n],[x0+n,y0+n]];
  shadowPoly(g,p,275);poly(g,p,'#7c3329');poly(g,[[x0+n+1,y0+1],[x0+w-n-1,y0+1],[x0+w-1,y0+n+1],[x0+w-1,y0+h-n-1],[x0+w-n-1,y0+h-1],[x0+n+1,y0+h-1],[x0+1,y0+h-n-1],[x0+1,y0+n+1]],'#8e3d31');
  shadowPoly(g,rectPts(x+2,y+s-10,s-4,8),20);roof(g,x+2,y+s-10,s-4,8,'#9c8f80');}
function bBrookfield(g,x,y,s){plaza(g,x,y,s,'#a7a296');const a=[x+2,y+2,17,19,260],b=[x+s-19,y+s-22,17,20,200];
  [a,b].forEach(t=>shadowPoly(g,rectPts(t[0],t[1],t[2],t[3]),t[4]));glassRoof(g,a[0],a[1],a[2],a[3],'#4c5866',1.4);glassRoof(g,b[0],b[1],b[2],b[3],'#6b7a86',1.4);
  const gx=x+21,gy=y+3,gw=7,gh=s-6;g.fillStyle='#d6e2e8';g.fillRect(gx,gy,gw,gh);g.strokeStyle='rgba(255,255,255,.85)';g.lineWidth=.12;for(let k=0;k<gh;k+=1.2){g.beginPath();g.moveTo(gx,gy+k);g.quadraticCurveTo(gx+gw/2,gy+k-.8,gx+gw,gy+k);g.stroke();}
  g.strokeStyle='rgba(80,90,100,.5)';g.lineWidth=.15;g.strokeRect(gx,gy,gw,gh);}
function bRTH(g,x,y,s){plaza(g,x,y,s,'#aba699');const cx=x+s*.45,cy=y+s*.55,R=17;
  g.fillStyle='rgba(12,14,10,.35)';g.beginPath();g.arc(cx+3,cy+3,R,0,7);g.fill();
  const gr=g.createRadialGradient(cx-5,cy-5,2,cx,cy,R);gr.addColorStop(0,'#c9dbe4');gr.addColorStop(1,'#5f7f93');g.fillStyle=gr;g.beginPath();g.arc(cx,cy,R,0,7);g.fill();
  g.strokeStyle='rgba(255,255,255,.35)';g.lineWidth=.1;for(let k=0;k<32;k++){const a=k/32*6.283;g.beginPath();g.moveTo(cx+Math.cos(a)*3,cy+Math.sin(a)*3);g.lineTo(cx+Math.cos(a)*R,cy+Math.sin(a)*R);g.stroke();}
  for(const r of [6,11,15]){g.beginPath();g.arc(cx,cy,r,0,7);g.stroke();}g.fillStyle='#d9d4c4';g.beginPath();g.arc(cx,cy,3,0,7);g.fill();
  shadowPoly(g,rectPts(x+s-15,y+2,13,16),110);glassRoof(g,x+s-15,y+2,13,16,'#7d8a93',1.4);
  for(let k=0;k<7;k++){const a=k/7*6.283+.4;tree(g,cx+Math.cos(a)*(R+3.5),cy+Math.sin(a)*(R+3.5),1.5);}}
function bCityHall(g,x,y,s){plaza(g,x,y,s,'#c2bcae');
  const px=x+6,py=y+s-17,pw=s-12,ph=8;g.fillStyle='#2f6f8a';g.fillRect(px,py,pw,ph);g.fillStyle='rgba(255,255,255,.18)';g.fillRect(px+1,py+1,pw*.5,.4);
  g.strokeStyle='#e8e6df';g.lineWidth=.6;for(let k=0;k<3;k++){const ax=px+pw*(k+.5)/3;g.beginPath();g.ellipse(ax,py+ph/2,pw/6-.6,ph/2+.6,0,0,7);g.stroke();}
  g.fillStyle='rgba(12,14,10,.35)';g.beginPath();g.arc(x+s/2+3,y+5,10,0,7);g.fill();
  g.fillStyle='#e4e2dc';g.beginPath();g.arc(x+s/2,y+2,9,0,7);g.fill();g.strokeStyle='#bdb9ae';g.lineWidth=.3;g.beginPath();g.arc(x+s/2,y+2,6,0,7);g.stroke();
  g.strokeStyle='#d7d5ce';g.lineWidth=5;g.beginPath();g.arc(x+s/2,y-14,24,.35,1.2);g.stroke();g.beginPath();g.arc(x+s/2,y-14,24,Math.PI-1.2,Math.PI-.35);g.stroke();
  for(let k=0;k<4;k++)tree(g,x+3+k*3,y+s-4,1.4);for(let k=0;k<4;k++)tree(g,x+s-3-k*3,y+s-4,1.4);}
function bOsgoode(g,x,y,s){g.fillStyle='#5f8a3c';g.fillRect(x,y,s,s);for(let k=0;k<s;k+=2.2){g.fillStyle='rgba(255,255,255,.04)';g.fillRect(x+k,y,1.1,s);}
  shadowPoly(g,rectPts(x+6,y+2,s-12,12),16);roof(g,x+6,y+2,s-12,12,'#cfc6b0');g.strokeStyle='#2a2a2a';g.lineWidth=.15;g.strokeRect(x+.6,y+.6,s-1.2,s-1.2);
  g.strokeStyle='#d9cfb2';g.lineWidth=1.4;g.beginPath();g.moveTo(x+s/2,y+s);g.lineTo(x+s/2,y+14);g.stroke();for(let k=0;k<8;k++)tree(g,x+sr(3,s-3),y+sr(17,s-3),sr(1.8,2.8));}
function bEaton(g,x,y,s){plaza(g,x,y,s,'#9f998d');shadowPoly(g,rectPts(x+1,y+1,s-12,s-2),35);roof(g,x+1,y+1,s-12,s-2,'#8e9399');
  const gx=x+s-11,gw=8;g.fillStyle='#d8e4ea';g.fillRect(gx,y,gw,s);g.strokeStyle='rgba(255,255,255,.9)';g.lineWidth=.12;for(let k=0;k<s;k+=1.1){g.beginPath();g.moveTo(gx,y+k);g.quadraticCurveTo(gx+gw/2,y+k-.7,gx+gw,y+k);g.stroke();}
  shadowPoly(g,rectPts(x+s-14,y+s-14,13,13),40);roof(g,x+s-14,y+s-14,13,13,'#cbbfa4');}
function bUnion(g,x,y,s){g.fillStyle='#9a958a';g.fillRect(x,y,s,12);shadowPoly(g,rectPts(x+1,y+1,s-2,10),25);roof(g,x+1,y+1,s-2,10,'#cbbfa4');g.fillStyle='#b8ab8f';g.fillRect(x+s/2-6,y+2,12,8);}
function drawSouthStrip(g){
  const y0=W+SW,yr=W+18,yr2=W+29;
  g.fillStyle='#6b6152';g.fillRect(-M,yr,W+2*M,yr2-yr);
  for(let k=0;k<5000;k++){g.fillStyle=SR()<.5?'rgba(0,0,0,.15)':'rgba(255,255,255,.06)';g.fillRect(sr(-M,W+M),sr(yr,yr2),.2,.2);}
  for(let t=0;t<5;t++){const ty=yr+1.4+t*2.1;g.fillStyle='rgba(60,45,30,.6)';for(let x=-M;x<W+M;x+=.6)g.fillRect(x,ty-.35,.22,1.6);g.fillStyle='#a4a6a8';g.fillRect(-M,ty,W+2*M,.09);g.fillRect(-M,ty+.9,W+2*M,.09);}
  g.fillStyle='rgba(0,0,0,.35)';g.fillRect(-M,yr-.6,W+2*M,.6);g.fillRect(-M,yr2,W+2*M,.6);
  g.fillStyle='#7d8a5c';g.fillRect(-M,yr2+.6,W+2*M,M);
  const sx=3*P-2,sw=P+4;g.fillStyle='#a9bcc6';g.fillRect(sx,yr,sw,yr2-yr);g.strokeStyle='rgba(255,255,255,.7)';g.lineWidth=.12;for(let k=0;k<sw;k+=1.3){g.beginPath();g.moveTo(sx+k,yr);g.lineTo(sx+k,yr2);g.stroke();}
  // CN Tower
  const cx=75,cy=W+36;g.fillStyle='rgba(10,12,10,.4)';g.beginPath();g.moveTo(cx-1.5,cy-1.5);g.lineTo(cx+60,cy+60);g.lineTo(cx+56,cy+64);g.lineTo(cx+1.5,cy+1.5);g.fill();
  g.beginPath();g.arc(cx+22,cy+22,8,0,7);g.fill();
  g.fillStyle='#bdb8ad';for(let k=0;k<3;k++){g.save();g.translate(cx,cy);g.rotate(k*2.094-1.57);g.fillRect(0,-1.2,11,2.4);g.restore();}
  g.fillStyle='#4a4f55';g.beginPath();g.arc(cx,cy,8,0,7);g.fill();g.fillStyle='#d9d4c8';g.beginPath();g.arc(cx,cy,6,0,7);g.fill();g.fillStyle='#8c939a';g.beginPath();g.arc(cx,cy,3.2,0,7);g.fill();g.fillStyle='#e8e4da';g.beginPath();g.arc(cx,cy,1.4,0,7);g.fill();
  // Rogers Centre dome edge
  const rx=10,ry=W+62,R=30;g.fillStyle='#c8cbcd';g.beginPath();g.arc(rx,ry,R,0,7);g.fill();g.strokeStyle='rgba(80,85,90,.6)';g.lineWidth=.3;for(const r of [R-4,R-9,R-15])g.beginPath(),g.arc(rx,ry,r,0,7),g.stroke();
  for(let k=0;k<16;k++){const a=k/16*6.283;g.beginPath();g.moveTo(rx,ry);g.lineTo(rx+Math.cos(a)*R,ry+Math.sin(a)*R);g.stroke();}
}
const TORONTO_BLOCKS=[ // [bj][bi] inside the grid
  ['heritage','heritage','osgoodepark','fin','fin'],
  ['condo','condo','fin','fin','fin'],
  ['park','condo','fin','fcp','scotia'],
  ['condo','rth','fin','td','fin'],
  ['lot','condo','fin','rbp','brook']];
function outerBlock(bi,bj){if(bj===-1){if(bi===2)return'osgoode';if(bi===3)return'cityhall';if(bi===4)return'eaton';return bi<2?'heritage':'fin';}
  if(bj===N)return bi===3?'union':'southlow';if(bi===-1)return'condo';if(bi===N)return'fin';return'condo';}
const BLOCKFN={fin:bFinancial,condo:bCondo,heritage:bHeritage,park:bPark,osgoodepark:bPark,lot:bParking,td:bTD,rbp:bRBP,fcp:bFCP,scotia:bScotia,brook:bBrookfield,rth:bRTH,cityhall:bCityHall,osgoode:bOsgoode,eaton:bEaton,union:bUnion,
  southlow:(g,x,y,s)=>{g.fillStyle='#8d877b';g.fillRect(x,y,s,11);const n=Math.floor(sr(2,4));let cx=x+1;for(let k=0;k<n;k++){const w=(s-2)/n-1;shadowPoly(g,rectPts(cx,y+1,w,8),15);roof(g,cx,y+1,w,8,pick(['#9c8f80','#8e9399','#a7a097'],SR));cx+=w+1;}}};
const STREETCAR_ROADS=[{h:true,r:0},{h:true,r:3},{h:false,r:0}];

function buildTexture(){
  TEX.width=TEX.height=TW;
  const g=TEX.getContext('2d');
  g.setTransform(TS,0,0,TS,M*TS,M*TS);
  g.fillStyle='#8f8a7f';g.fillRect(-M,-M,W+2*M,W+2*M);
  // sidewalks
  g.fillStyle='#aaa79e';
  for(let i=0;i<=N;i++){g.fillRect(i*P-SW,-M,2*SW,W+2*M);g.fillRect(-M,i*P-SW,W+2*M,2*SW);}
  g.fillStyle='rgba(0,0,0,.13)';
  for(let i=0;i<=N;i++)for(let t=-M;t<W+M;t+=1.6)for(const o of [-SW,RH]){g.fillRect(i*P+o,t,SW-RH,.06);g.fillRect(t,i*P+o,.06,SW-RH);}
  // asphalt
  g.fillStyle='#3b3e42';
  for(let i=0;i<=N;i++){g.fillRect(i*P-RH,-M,2*RH,W+2*M);g.fillRect(-M,i*P-RH,W+2*M,2*RH);}
  for(let k=0;k<90;k++){const i=Math.floor(SR()*(N+1)),t=sr(-M,W+M),o=sr(-RH+1,RH-2),v=SR()<.5,w=sr(1,4),h=sr(.8,2.5);
    g.fillStyle='rgba(72,75,79,.7)';v?g.fillRect(i*P+o,t,h,w):g.fillRect(t,i*P+o,w,h);}
  for(let k=0;k<26000;k++){const i=Math.floor(SR()*(N+1)),t=sr(-M,W+M),o=sr(-RH,RH),v=SR()<.5;
    g.fillStyle=SR()<.55?'rgba(0,0,0,.2)':'rgba(255,255,255,.07)';const sz=sr(.1,.3);v?g.fillRect(i*P+o,t,sz,sz):g.fillRect(t,i*P+o,sz,sz);}
  g.fillStyle='rgba(0,0,0,.09)';
  for(let i=0;i<=N;i++)for(const o of [-2.75,-1.25,1.25,2.75]){g.fillRect(i*P+o-.25,-M,.5,W+2*M);g.fillRect(-M,i*P+o-.25,W+2*M,.5);}
  // markings
  for(let j=0;j<=N;j++)for(let i=-1;i<=N;i++){
    const a=Math.max(-M,i*P+7.6),b=Math.min(W+M,(i+1)*P-7.6),L=b-a;if(L<=0)continue;
    g.fillStyle='#d8b13c';g.fillRect(a,j*P-.25,L,.13);g.fillRect(a,j*P+.12,L,.13);g.fillRect(j*P-.25,a,.13,L);g.fillRect(j*P+.12,a,.13,L);
    g.fillStyle='rgba(235,235,230,.85)';
    g.fillRect(a,j*P-3.75,L,.12);g.fillRect(a,j*P+3.63,L,.12);g.fillRect(j*P-3.75,a,.12,L);g.fillRect(j*P+3.63,a,.12,L);
    if(i<N){g.fillRect(b-.45,j*P+.3,.45,3.4);g.fillRect(j*P-3.7,b-.45,3.4,.45);}
    if(i>=0){g.fillRect(a,j*P-3.7,.45,3.4);g.fillRect(j*P+.3,a,3.4,.45);}
  }
  g.fillStyle='rgba(238,238,232,.85)';
  for(let i=0;i<=N;i++)for(let j=0;j<=N;j++)for(const sg of [-1,1]){
    for(let t=-3.5;t<3.5;t+=.9)g.fillRect(i*P+t,j*P+sg*NODE-.9,.5,1.8);
    for(let t=-3.5;t<3.5;t+=.9)g.fillRect(i*P+sg*NODE-.9,j*P+t,1.8,.5);}
  // TTC streetcar tracks (Queen, King, Spadina)
  for(const sc of STREETCAR_ROADS)for(const lo of [-2,2])for(const rg of [-.72,.72]){const c=sc.r*P+lo+rg;
    g.fillStyle='rgba(20,22,24,.75)';sc.h?g.fillRect(-M,c-.07,W+2*M,.14):g.fillRect(c-.07,-M,.14,W+2*M);
    g.fillStyle='rgba(170,175,180,.55)';sc.h?g.fillRect(-M,c-.02,W+2*M,.04):g.fillRect(c-.02,-M,.04,W+2*M);}
  // blocks
  const typeGrid=[];
  for(let bj=-1;bj<=N;bj++)for(let bi=-1;bi<=N;bi++){
    const inner=bi>=0&&bi<N&&bj>=0&&bj<N,pyB=PY.map&&PY.map.blocks[bi+','+bj],t=(pyB&&BLOCKFN[pyB.style])?pyB.style:(inner?TORONTO_BLOCKS[bj][bi]:outerBlock(bi,bj)); // block styles come from the Python map (BeautifulSoup) when available
    if(inner)typeGrid.push(t==='osgoodepark'?'park':t);
    const x0=bi*P+SW,y0=bj*P+SW,s=P-2*SW;
    g.fillStyle='#8a857a';g.fillRect(x0,y0,s,s);
    g.strokeStyle='#c9c6bc';g.lineWidth=.22;g.strokeRect(x0+.1,y0+.1,s-.2,s-.2);
    BLOCKFN[t](g,x0+1,y0+1,s-2);
    const pr=t==='park'||t==='osgoodepark'?.75:t==='heritage'||t==='condo'?.45:.3;
    if(bj<N)for(let side=0;side<4;side++)for(let u=4;u<s-4;u+=sr(7,10)){if(SR()>pr)continue;
      const tx=side===0?x0+u:side===1?x0+s-.6:side===2?x0+u:x0+.6, ty=side===0?y0+.6:side===1?y0+u:side===2?y0+s-.6:y0+u;
      tree(g,tx,ty,sr(1.7,2.5));}
  }
  drawSouthStrip(g);
  for(const z of wetZones){const gr=g.createRadialGradient(z.x,z.y,z.r*.2,z.x,z.y,z.r);gr.addColorStop(0,'rgba(18,32,52,.55)');gr.addColorStop(.75,'rgba(18,32,52,.4)');gr.addColorStop(1,'rgba(18,32,52,0)');
    g.fillStyle=gr;g.beginPath();g.arc(z.x,z.y,z.r,0,7);g.fill();
    for(let k=0;k<6;k++){g.fillStyle='rgba(170,200,230,.12)';g.beginPath();g.ellipse(z.x+sr(-z.r*.5,z.r*.5),z.y+sr(-z.r*.5,z.r*.5),sr(.6,2),sr(.15,.4),sr(0,3),0,7);g.fill();}}
  for(let k=0;k<60000;k++){g.fillStyle=SR()<.5?'rgba(0,0,0,.06)':'rgba(255,255,255,.04)';g.fillRect(sr(-M,W+M),sr(-M,W+M),.14,.14);}
  paintControls(g);   // stop lines, yield teeth, STOP lettering (13-traffic.js)
  miniBase=document.createElement('canvas');miniBase.width=miniBase.height=340;
  miniBase.getContext('2d').drawImage(TEX,0,0,TW,TW,0,0,340,340);
  return typeGrid;
}

