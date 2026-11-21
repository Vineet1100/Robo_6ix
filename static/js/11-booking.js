/* =============================================================================
 * 11-booking.js — Booking map (A → B)
 * The full-map ride booking dialog: snapping pickup/drop-off, route preview, confirm.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== booking map (pickup → destination) ==============================
const bk={open:false,step:1,pick:null,dest:null,hover:null,base:null,wasPaused:false,r1:null,r2:null};
const BM=1520,bmS=BM/(W+2*M),b2w=(px,py)=>[px/bmS-M,py/bmS-M],w2b=(x,y)=>[(x+M)*bmS,(y+M)*bmS];
function snapPickup(x,y){const vi=Math.round(x/P),hj=Math.round(y/P),dv=Math.abs(x-vi*P),dh=Math.abs(y-hj*P);let p;
  if(dh<=dv){const k=clamp(Math.floor(x/P),0,N-1);p={x:clamp(x,k*P+9,k*P+P-9),y:clamp(hj,0,N)*P+(y>=hj*P?1:-1)*NODE};}
  else{const k=clamp(Math.floor(y/P),0,N-1);p={x:clamp(vi,0,N)*P+(x>=vi*P?1:-1)*NODE,y:clamp(y,k*P+9,k*P+P-9)};}
  return(p.x<0||p.x>W||p.y<0||p.y>W)?null:p;}
function inService(x,y){return x>-8&&x<W+8&&y>-8&&y<W+8;}
function openBooking(){if(!miniBase)return;bk.open=true;bk.step=1;bk.pick=null;bk.dest=null;bk.r1=bk.r2=null;bk.wasPaused=paused;paused=true;
  if(!bk.base){bk.base=document.createElement('canvas');bk.base.width=bk.base.height=BM;const g=bk.base.getContext('2d');g.drawImage(TEX,0,0,TW,TW,0,0,BM,BM);
    g.fillStyle='rgba(0,0,0,.18)';g.fillRect(0,0,BM,BM);g.textAlign='center';g.font='700 21px Inter';
    for(let r=0;r<=N;r++)for(const k of [0,2,4])for(const hz of [true,false]){const along=k*P+P/2,[x,y]=w2b(hz?along:r*P,hz?r*P:along);g.save();g.translate(x,y);if(!hz)g.rotate(-Math.PI/2);
      const t=hz?HNAME[r]:VNAME[r];g.lineWidth=5;g.strokeStyle='rgba(8,10,12,.85)';g.strokeText(t,0,7);g.fillStyle='#fff';g.fillText(t,0,7);g.restore();}
    g.font='700 20px Inter';const lm=PY.map?PY.map.pois.map(p=>[p.name,p.x,p.y]):[['CN Tower',75,W+28],['Union Station',210,W+12],['Nathan Phillips Sq.',210,-24],['Roy Thomson Hall',80,214],['TD Centre',200,196],['Eaton Centre',270,-22],['Osgoode Hall',150,-30]];
    for(const[t,x,y]of lm){const[px,py]=w2b(x,y);g.lineWidth=5;g.strokeStyle='rgba(8,10,12,.85)';g.strokeText(t,px,py);g.fillStyle='#ffe08a';g.fillText(t,px,py);}
    const[a0]=w2b(0,0),[a1]=w2b(W,W);g.strokeStyle='rgba(56,225,198,.8)';g.lineWidth=4;g.setLineDash([16,10]);g.strokeRect(a0-14,a0-14,a1-a0+28,a1-a0+28);g.setLineDash([]);
    g.textAlign='left';g.fillStyle='rgba(56,225,198,.95)';g.font='700 18px Inter';g.fillText('SERVICE AREA',a0-10,a0-22);}
  $('book').hidden=false;bookUI();drawBooking();}
function closeBooking(){bk.open=false;$('book').hidden=true;paused=bk.wasPaused;}
function bkEta(len,turns=0,ctrls=0){const m=PY.etaModel,t=m?Math.max(5,m.base_s+m.per_m*len+m.per_turn*turns+(m.per_ctrl||0)*ctrls):len/6.5+20+4*turns+6*ctrls;  // ETA model fitted in Python (scikit-learn) once 3+ legs are logged
  return `${Math.floor(t/60)}:${String(Math.round(t%60)).padStart(2,'0')}`;}
function bookUI(){
  $('bStep').textContent=bk.step===1?'Step 1 of 2 · Tap the map where you want to be picked up':bk.dest?'Ready · Check the route and confirm':'Step 2 of 2 · Now tap where you want to go';
  $('bA').classList.toggle('act',bk.step===1);$('bB').classList.toggle('act',bk.step===2);
  $('bAtxt').textContent=bk.pick?destName(destFromPoint(bk.pick.x,bk.pick.y)):'Tap the map';
  $('bBtxt').textContent=bk.dest?destName(bk.dest):(bk.step===2?'Tap the map':'Choose after pickup');
  let info=(PY.etaModel?'ETAs from a scikit-learn model trained on this session\'s trips. ':'')+'Pickups snap to the nearest sidewalk and drop-offs to the nearest curb lane. Spots inside the outlined service area work best.';
  if(bk.pick&&bk.r1)info=`🚕 Taxi to you: <b>${bk.r1.len.toFixed(0)} m</b>, about <b>${bkEta(bk.r1.len,bk.r1.nodes.filter(n=>n.turn).length,ctrlCount(bk.r1))}</b>${ride.phase!=='idle'&&ride.phase!=='toPickup'?' after its current drop-off':''}.`;
  if(bk.dest&&bk.r2)info+=`<br>🧭 Your trip: <b>${bk.r2.len.toFixed(0)} m</b>, about <b>${bkEta(bk.r2.len,bk.r2.nodes.filter(n=>n.turn).length,ctrlCount(bk.r2))}</b>, ${bk.r2.nodes.filter(n=>n.turn).length} turns.`;
  if(bk.dest&&!bk.r2)info+='<br><span style="color:#ffb020">No legal route to that drop-off — try another spot.</span>';
  $('bInfo').innerHTML=info;$('bGo').disabled=!(bk.pick&&bk.dest&&bk.r2);}
function drawBooking(){
  const c=$('bmap').getContext('2d');c.setTransform(1,0,0,1,0,0);c.drawImage(bk.base,0,0);
  const line=(r,col,w,dash)=>{if(!r)return;c.strokeStyle=col;c.lineWidth=w;c.setLineDash(dash||[]);c.lineJoin='round';c.lineCap='round';c.beginPath();for(let q=0;q<r.n;q+=2){const[x,y]=w2b(r.px[q],r.py[q]);q?c.lineTo(x,y):c.moveTo(x,y);}const[x,y]=w2b(r.px[r.n-1],r.py[r.n-1]);c.lineTo(x,y);c.stroke();c.setLineDash([]);};
  line(bk.r1,'rgba(56,225,198,.9)',6,[14,10]);line(bk.r2,'rgba(10,20,40,.7)',14);line(bk.r2,'#4f8dff',8);
  {const[x,y]=w2b(ego.x,ego.y);c.save();c.translate(x,y);c.rotate(ego.th);c.fillStyle='#38e1c6';c.beginPath();c.moveTo(20,0);c.lineTo(-13,-12);c.lineTo(-6,0);c.lineTo(-13,12);c.closePath();c.fill();c.restore();
   c.font='700 18px Inter';c.lineWidth=5;c.strokeStyle='rgba(8,10,12,.9)';c.strokeText('TAXI',x+16,y-16);c.fillStyle='#38e1c6';c.fillText('TAXI',x+16,y-16);}
  const pin=(x,y,col,lab)=>{const[px,py]=w2b(x,y);c.fillStyle='rgba(0,0,0,.35)';c.beginPath();c.ellipse(px+4,py+2,12,5,0,0,7);c.fill();
    c.fillStyle=col;c.beginPath();c.moveTo(px,py);c.bezierCurveTo(px-20,py-26,px-18,py-50,px,py-50);c.bezierCurveTo(px+18,py-50,px+20,py-26,px,py);c.fill();
    c.strokeStyle='#fff';c.lineWidth=3;c.stroke();c.fillStyle='#fff';c.font='800 20px Inter';c.textAlign='center';c.fillText(lab,px,py-27);c.textAlign='left';};
  if(bk.hover){const h=bk.hover;c.globalAlpha=.55;pin(h.x,h.y,bk.step===1?'#1fb37a':'#e5484d',bk.step===1?'A':'B');c.globalAlpha=1;}
  if(bk.pick){pin(bk.pick.x,bk.pick.y,'#1fb37a','A');const[px,py]=w2b(bk.pick.x,bk.pick.y);c.strokeStyle='rgba(31,179,122,.7)';c.lineWidth=3;for(let k=1;k<=2;k++){c.beginPath();c.arc(px,py,10+k*12,0,7);c.stroke();}}
  if(bk.dest){const d=destPoint(bk.dest);pin(d[0],d[1],'#e5484d','B');}
}
function bkPoint(e){const b=$('bmap').getBoundingClientRect();return b2w((e.clientX-b.left)/b.width*BM,(e.clientY-b.top)/b.height*BM);}
function setupBooking(){
  const cvs=$('bmap');
  cvs.addEventListener('mousemove',e=>{const[x,y]=bkPoint(e);let h=null,txt='';
    if(bk.step===1){const p=snapPickup(x,y);if(p&&inService(x,y)){h=p;txt='Pickup here: '+destName(destFromPoint(p.x,p.y));}else txt='Outside the service area';}
    else{if(inService(x,y)){const d=destFromPoint(x,y),pt=destPoint(d);h={x:pt[0],y:pt[1]};txt='Drop-off here: '+destName(d);}else txt='Outside the service area';}
    bk.hover=h;$('bHover').textContent=txt;drawBooking();});
  cvs.addEventListener('mouseleave',()=>{bk.hover=null;$('bHover').textContent='';drawBooking();});
  cvs.addEventListener('click',e=>{const[x,y]=bkPoint(e);if(!inService(x,y))return;
    if(bk.step===1){const p=snapPickup(x,y);if(!p)return;bk.pick=p;const pd=destFromPoint(p.x,p.y);bk.r1=planRoute(ego.x,ego.y,ego.d,pd);bk.step=2;bk.dest=null;bk.r2=null;}
    else{const d=destFromPoint(x,y);bk.dest=d;const pd=destFromPoint(bk.pick.x,bk.pick.y),lp=destPoint(pd);let r=planRoute(lp[0],lp[1],pd.dd,d);if(r&&r.len<25)r=null;bk.r2=r;}
    bookUI();drawBooking();});
  $('bBack').onclick=()=>{if(bk.step===2){bk.step=1;bk.dest=null;bk.r2=null;}else{bk.pick=null;bk.r1=null;}bookUI();drawBooking();};
  $('bClose').onclick=closeBooking;$('book').addEventListener('click',e=>{if(e.target.id==='book')closeBooking();});
  window.addEventListener('keydown',e=>{if(bk.open&&e.key==='Escape')closeBooking();});
  $('bGo').onclick=()=>{if(!(bk.pick&&bk.dest&&bk.r2))return;const p=bk.pick,d=bk.dest;closeBooking();paused=false;$('tgPause').classList.remove('on');
    ride.auto=false;$('tgAuto').classList.remove('on');makeRequest(p.x,p.y,{dest:d,user:true});};
}
