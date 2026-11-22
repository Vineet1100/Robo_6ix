/* =============================================================================
 * 02-texture.js — Aerial imagery of downtown Toronto, plus how cars and streetcars are drawn
 * The satellite-style city (roads, markings, crosswalks, TTC tracks, trees, roofs, shadows, wet patches,
 * traffic-control markings and every landmark) is drawn in Python by robotaxi/texture.py with Pillow and
 * NumPy, and served by Flask at /api/texture.jpg. This file loads that image. Opened without the server,
 * it draws a plain street map instead so the simulation still runs.
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
// block styles when the Python map isn't available (offline mode); [bj][bi]
const TORONTO_BLOCKS=[
  ['heritage','heritage','osgoodepark','fin','fin'],
  ['condo','condo','fin','fin','fin'],
  ['park','condo','fin','fcp','scotia'],
  ['condo','rth','fin','td','fin'],
  ['lot','condo','fin','rbp','brook']];
const FLAT={park:'#5f8a3c',osgoodepark:'#5f8a3c',lot:'#55595e',condo:'#a07a68',heritage:'#9a7462',fin:'#7d8790'};

// Load the Python-drawn aerial image into the TEX canvas (or draw the offline street map).
// Returns the style of each inner block, which initWorld() uses to put raccoons in the parks.
async function buildTexture(){
  TEX.width=TEX.height=TW;
  const g=TEX.getContext('2d');
  const typeGrid=[];
  for(let bj=0;bj<N;bj++)for(let bi=0;bi<N;bi++){const pyB=PY.map&&PY.map.blocks[bi+','+bj],t=pyB?pyB.style:TORONTO_BLOCKS[bj][bi];typeGrid.push(t==='osgoodepark'?'park':t);}
  let ok=false;
  if(PY.online){
    try{const img=new Image();img.src='api/texture.jpg';await img.decode();g.drawImage(img,0,0,TW,TW);ok=true;}
    catch(e){console.warn('aerial image unavailable, drawing the plain map',e);}
  }
  if(!ok)drawPlainMap(g,typeGrid);
  miniBase=document.createElement('canvas');miniBase.width=miniBase.height=340;
  miniBase.getContext('2d').drawImage(TEX,0,0,TW,TW,0,0,340,340);
  return typeGrid;
}
// Offline fallback: flat blocks, sidewalks, asphalt, centre lines and crosswalks.
function drawPlainMap(g,typeGrid){
  g.setTransform(TS,0,0,TS,M*TS,M*TS);
  g.fillStyle='#8f8a7f';g.fillRect(-M,-M,W+2*M,W+2*M);
  for(let bj=0;bj<N;bj++)for(let bi=0;bi<N;bi++){g.fillStyle=FLAT[typeGrid[bj*N+bi]]||'#7d8790';g.fillRect(bi*P+SW+1,bj*P+SW+1,P-2*SW-2,P-2*SW-2);}
  g.fillStyle='#aaa79e';for(let i=0;i<=N;i++){g.fillRect(i*P-SW,-M,2*SW,W+2*M);g.fillRect(-M,i*P-SW,W+2*M,2*SW);}
  g.fillStyle='#3b3e42';for(let i=0;i<=N;i++){g.fillRect(i*P-RH,-M,2*RH,W+2*M);g.fillRect(-M,i*P-RH,W+2*M,2*RH);}
  g.fillStyle='#d8b13c';for(let j=0;j<=N;j++)for(let i=-1;i<=N;i++){const a=Math.max(-M,i*P+7.6),L=Math.min(W+M,(i+1)*P-7.6)-a;if(L<=0)continue;
    g.fillRect(a,j*P-.2,L,.15);g.fillRect(j*P-.2,a,.15,L);}
  g.fillStyle='rgba(238,238,232,.85)';for(let i=0;i<=N;i++)for(let j=0;j<=N;j++)for(const sg of [-1,1])for(let t=-3.5;t<3.5;t+=.9){
    g.fillRect(i*P+t,j*P+sg*NODE-.9,.5,1.8);g.fillRect(i*P+sg*NODE-.9,j*P+t,1.8,.5);}
  for(const z of wetZones){g.fillStyle='rgba(18,32,52,.35)';g.beginPath();g.arc(z.x,z.y,z.r,0,7);g.fill();}
  g.setTransform(1,0,0,1,0,0);
}
