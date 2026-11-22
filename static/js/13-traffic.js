/* =============================================================================
 * 13-traffic.js — Traffic lights, stop signs, yield signs and speed limits
 * Which intersection has which control, signal timing, the right-of-way rules that every car
 * (and the robotaxi) follows, pedestrian walk signals, road markings and the signs drawn on the map.
 * The layout and speed limits come from data/toronto_downtown.osm via Python when the server is
 * running; the defaults below are the same layout, used when the page is opened on its own.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== layout ==============================
let SPEED_H=[40,40,40,40,40,50];               // km/h, Queen → Front
let SPEED_V=[50,30,50,30,40,40];               // km/h, Spadina → Yonge
const STOPL=7.6;                                // stop line distance from the centre of an intersection (m)
const SIGT={G:18,Y:3,AR:2};                      // seconds of green, amber and all-red per direction
const CYCLE=2*(SIGT.G+SIGT.Y+SIGT.AR);             // 46 s
const WALK_MIN=9;                               // a pedestrian only starts crossing with this much green left
const CTRL={};                                  // "i,j" → {i,j,key,type,approach,offset,q,name}
const CTRL_NAME={signal:'Traffic light',allstop:'All-way stop',stop:'Stop sign',yield:'Yield sign'};
const rules={redStops:0,stopSigns:0,yields:0,violations:0,overT:0,driveT:0,maxOver:0};

function defaultControl(i,j){            // same layout as the map file
  const mh=[0,3,5].includes(j),mv=[0,2,4,5].includes(i);
  if(mh&&mv)return{type:'signal',offset:(i*11+j*17)%46};
  if(!mh&&!mv)return{type:'allstop'};
  if(mh)return{type:'stop',approach:'v'};
  if(j===1||j===2)return{type:'signal',offset:(i*11+j*17)%46};
  return{type:'yield',approach:'h'};
}
function setupTraffic(){
  if(PY.map&&PY.map.maxspeed_h&&PY.map.maxspeed_h.length===N+1){SPEED_H=PY.map.maxspeed_h.slice();SPEED_V=PY.map.maxspeed_v.slice();}
  VLIM=Math.max(...SPEED_H,...SPEED_V)/3.6;    // global cap used by the PID clamps; each street still has its own limit
  for(let i=0;i<=N;i++)for(let j=0;j<=N;j++){const key=i+','+j;
    const src=PY.map&&PY.map.controls?PY.map.controls[key]:defaultControl(i,j);if(!src)continue;
    CTRL[key]=Object.assign({i,j,key,q:[],offset:0,approach:null},src,{name:(PY.map&&PY.map.intersections[key])||`${HNAME[j]} & ${VNAME[i]}`});}
}
const ctrlAt=(i,j)=>CTRL[i+','+j];
const axisOf=d=>horiz(d)?'h':'v';
const limitKmh=(d,x,y)=>horiz(d)?SPEED_H[clamp(Math.round(y/P),0,N)]:SPEED_V[clamp(Math.round(x/P),0,N)];
function approachRule(c,axis){
  if(c.type==='signal')return'signal';
  if(c.type==='allstop')return'stop';
  return c.approach===axis?c.type:'free';
}
function sigState(c,axis,t=simT){          // {s:'G'|'Y'|'R', rem: seconds until it changes}
  const u=((t+c.offset)%CYCLE+CYCLE)%CYCLE,h=SIGT.G+SIGT.Y+SIGT.AR,w=axis==='h'?u:(u-h+CYCLE)%CYCLE;
  if(w<SIGT.G)return{s:'G',rem:SIGT.G-w};
  if(w<SIGT.G+SIGT.Y)return{s:'Y',rem:SIGT.G+SIGT.Y-w};
  return{s:'R',rem:CYCLE-w};
}
// ============================== right of way ==============================
const inBox=(o,c,m=STOPL)=>Math.abs(o.x-c.i*P)<m&&Math.abs(o.y-c.j*P)<m;
function approachInfo(o,c){                 // how far a vehicle is from the centre of c along its heading (+ = approaching)
  const d=o.d,hx=horiz(d),perp=hx?o.y-c.j*P:o.x-c.i*P;
  return{d,axis:axisOf(d),perp,dI:sgnD(d)*(hx?c.i*P-o.x:c.j*P-o.y)};
}
const isVeh=o=>o.kind==='car'||o.kind==='ego';
// Is it safe to enter c from the given axis? Checks the box itself and cross traffic that does not have to stop.
function crossClear(c,axis,self,horizon){
  for(const o of actors){if(o===self||!isVeh(o))continue;if(Math.abs(o.x-c.i*P)>65||Math.abs(o.y-c.j*P)>65)continue;
    if(inBox(o,c,STOPL-.6)){if(o.v>.2||!(o.cs&&o.cs.key===c.key&&!o.cs.go))return false;continue;}
    const A=approachInfo(o,c);if(A.axis===axis||Math.abs(A.perp)>4.5||A.dI<0)continue;
    const r=approachRule(c,A.axis);
    if(r==='free'||r==='signal'){if(o.v>.5&&A.dI-STOPL<Math.max(o.v,2.5)*horizon+5)return false;}
    else if(o.cs&&o.cs.key===c.key&&o.cs.go&&A.dI<18)return false;
  }
  return true;
}
// Left turn on a green: wait for oncoming traffic that is going straight or turning right.
function oncomingClear(c,self){
  const sd=self.d;
  for(const o of actors){if(o===self||!isVeh(o))continue;if(Math.abs(o.x-c.i*P)>65||Math.abs(o.y-c.j*P)>65)continue;
    if(o.d!==(sd+2)%4)continue;const A=approachInfo(o,c);if(Math.abs(A.perp)>4.5||A.dI<-2)continue;
    if(turnAtNode(o,c)==='left')continue;
    if(inBox(o,c)&&o.v>.2)return false;
    if(o.v>.5&&A.dI-STOPL<Math.max(o.v,3)*4+4)return false;
    if(A.dI-STOPL<5&&((o.cs&&o.cs.key===c.key&&o.cs.go)||(c.type==='signal'&&sigState(c,axisOf(o.d)).s==='G')))return false;}   // at its line with a green / its turn
  return true;
}
// An oncoming vehicle that has already been given its turn to go left across our path.
function oncomingLeftGo(c,self){
  for(const o of actors){if(o===self||!isVeh(o)||o.d!==(self.d+2)%4)continue;const A=approachInfo(o,c);
    if(o.cs&&o.cs.key===c.key&&o.cs.go&&Math.abs(A.perp)<4.5&&A.dI>-2&&A.dI<18&&turnAtNode(o,c)==='left')return true;}
  return false;
}
function turnAtNode(o,c){
  if(o.kind==='ego'){const nd=route&&route.nodes.find(n=>n.i===c.i&&n.j===c.j&&n.q>=ego.pi-30);return nd?nd.turn:null;}
  const p=o.plan;if(!p)return null;const A=approachInfo(o,c),cr=horiz(o.d)?c.i*P:c.j*P;if(Math.abs(p.cross-cr)>1||A.dI<-1)return null;
  return p.d2===(o.d+3)%4?'left':p.d2===(o.d+1)%4?'right':null;
}
// The rule engine shared by every car and by the robotaxi. g = distance from the front bumper to the stop line.
function ctrlDecide(a,c,axis,g,turn,visit){   // visit: tells repeat visits to one intersection apart (looping routes)
  const rule=approachRule(c,axis);
  if(rule==='free'&&turn!=='left')return{hold:false,rule};
  if(!a.cs||a.cs.key!==c.key||a.cs.visit!==visit||simT-(a.cs.tSeen??simT)>5)a.cs={key:c.key,visit,t0:-1,done:false,go:false,ygo:null,tDone:0,lw:false};
  const cs=a.cs,v=a.v;cs.tSeen=simT;
  if(rule==='free'){                                                        // main road, turning left: wait for oncoming traffic
    if(cs.go)return{hold:false,rule};if(oncomingClear(c,a)){if(g<3)cs.go=true;return{hold:false,rule};}return{hold:true,rule,why:'left'};}
  if(rule==='signal'){
    const st=sigState(c,axis);
    if(st.s==='G'){cs.ygo=null;
      if(turn==='left'&&!cs.go){if(oncomingClear(c,a)){if(g<3)cs.go=true;return{hold:false,rule,st};}cs.lw=true;return{hold:true,rule,st,why:'left'};}
      return{hold:false,rule,st};}
    if(cs.go)return{hold:false,rule,st};                                     // already committed to a left turn
    if(st.s==='Y'){
      if(cs.ygo===null)cs.ygo=g<v*v/(2*3.2)+.8;                              // can't stop comfortably → carry on
      if(turn==='left'&&cs.lw&&v<.5&&g<4&&oncomingClear(c,a)){cs.go=true;return{hold:false,rule,st};}
      return{hold:!cs.ygo,rule,st,why:'amber'};}
    if(cs.ygo)return{hold:false,rule,st};
    return{hold:true,rule,st,why:'red'};
  }
  if(rule==='stop'){
    if(!cs.done){
      if(v<.3&&g<4.5){if(cs.t0<0)cs.t0=simT;if(simT-cs.t0>=1){cs.done=true;cs.tDone=simT;c.q.push(a);}}else cs.t0=-1;
      return{hold:true,rule,why:'stop'};}
    if(cs.go)return{hold:false,rule};
    c.q=c.q.filter(o=>o.cs&&o.cs.key===c.key&&!o.cs.go&&actors.includes(o));
    const qok=c.type!=='allstop'||c.q[0]===a||simT-cs.tDone>6;
    // after 8 s of waiting at the line, accept a tighter gap in cross traffic
    const clear=crossClear(c,axis,a,simT-cs.tDone>8?2.3:3.5)&&(turn!=='left'||oncomingClear(c,a))&&!oncomingLeftGo(c,a);
    if(qok&&clear){cs.go=true;c.q=c.q.filter(o=>o!==a);return{hold:false,rule};}
    return{hold:true,rule,why:c.type==='allstop'&&!qok?'queue':'gap'};
  }
  // yield
  if(cs.go)return{hold:false,rule};
  if(crossClear(c,axis,a,4)&&(turn!=='left'||oncomingClear(c,a))){if(g<6)cs.go=true;return{hold:false,rule};}
  return{hold:true,rule,why:'yield'};
}
// NPC cars and streetcars: distance to a stop line they must hold at (1e9 if none).
function npcStopGap(a){
  if(a.noYield)return 1e9;                                   // the scripted red-light runner ignores everything
  const s=sgnD(a.d),hx=horiz(a.d),al=hx?a.x:a.y,perp=hx?a.y:a.x;
  const cr=s>0?Math.ceil(al/P)*P:Math.floor(al/P)*P,rc=Math.round(perp/P)*P,dI=s*(cr-al);
  if(dI>70||cr<0||cr>W)return 1e9;
  const c=hx?ctrlAt(cr/P,rc/P):ctrlAt(rc/P,cr/P);if(!c)return 1e9;
  const g=dI-STOPL-a.L/2;if(g<-.6)return 1e9;              // past the line: committed
  return ctrlDecide(a,c,axisOf(a.d),g,turnAtNode(a,c)).hold?Math.max(g,.05):1e9;
}
// ============================== the robotaxi ==============================
// Adds stop-line index (qs), approach heading and control to each intersection on a route, plus a per-point speed limit.
function annotateRoute(r){
  r.vl=new Float32Array(r.n);
  for(let q=0;q<r.n;q++){const tx=Math.abs(r.tx[q]),ty=Math.abs(r.ty[q]),kh=SPEED_H[clamp(Math.round(r.py[q]/P),0,N)],kv=SPEED_V[clamp(Math.round(r.px[q]/P),0,N)];
    r.vl[q]=(tx>.92?kh:ty>.92?kv:Math.min(kh,kv))/3.6;}
  for(const nd of r.nodes){const cx=nd.i*P,cy=nd.j*P;let q=nd.q;
    while(q>0&&Math.max(Math.abs(r.px[q]-cx),Math.abs(r.py[q]-cy))<STOPL)q--;
    nd.qs=q;const tx=r.tx[Math.max(0,q-1)],ty=r.ty[Math.max(0,q-1)];nd.ad=Math.abs(tx)>Math.abs(ty)?(tx>0?0:2):(ty>0?1:3);nd.ctrl=ctrlAt(nd.i,nd.j)||null;}
}
// Called every step from egoNominal(): returns the acceleration limit the traffic rules impose.
function egoRules(dt,v){
  const r=route;ego.vlim=r&&r.vl?r.vl[Math.min(ego.pi,r.n-1)]:limitKmh(ego.d,ego.x,ego.y)/3.6;
  let acc=1e9;ego.ctl=null;if(!r||!r.vl)return acc;
  const q0=Math.min(ego.pi,r.n-1),sOff=(ego.x-r.px[q0])*r.tx[q0]+(ego.y-r.py[q0])*r.ty[q0];   // sub-sample position along the path
  for(const nd of r.nodes){const c=nd.ctrl;if(!c)continue;
    const g=(nd.qs-ego.pi)*DS-sOff-ego.L/2;if(g<-.6){                      // crossed this stop line
      if(!nd.passed){nd.passed=true;const cs=ego.cs&&ego.cs.key===c.key&&ego.cs.visit===nd.q?ego.cs:null,rl=approachRule(c,axisOf(nd.ad));
        if(cs&&!cs.passed){cs.passed=true;
        if(rl==='signal'&&sigState(c,axisOf(nd.ad)).s==='R'&&!(cs&&(cs.ygo||cs.go))){rules.violations++;logE(`🚨 Entered ${c.name} on a red light`,'bad');}
        if(rl==='stop'&&!cs.done){rules.violations++;logE(`🚨 Rolled through the stop sign at ${c.name}`,'bad');}}}
      continue;}
    if(g>75)break;
    const res=ctrlDecide(ego,c,axisOf(nd.ad),g,nd.turn,nd.q);
    ego.ctl={c,g,nd,res,rule:res.rule,st:res.st,hold:res.hold,why:res.why};
    if(res.hold){const gs=g-1.2;                                       // aim to stop 1.2 m before the line
      acc=Math.min(acc,gs<=0?(v>.05?-3.5:-1):clamp((Math.sqrt(2*2.2*gs)-v)*1.8,-6.5,1.2));
      const cs=ego.cs;
      if(ego.v<.3&&cs&&cs.key===c.key&&!cs.counted&&res.why!=='stop'&&res.why!=='queue'&&res.why!=='gap'){cs.counted=true;
        if(res.why==='red'||res.why==='amber'){rules.redStops++;logE(`🚦 Stopped for a ${res.why==='red'?'red':'amber'} light at ${c.name}`,'info');}
        else if(res.why==='left'||res.why==='yield'){rules.yields++;logE(`⚠️ Yielding at ${c.name}${res.why==='left'?' (left turn, oncoming traffic)':''}`,'info');}}}
    if(res.rule==='stop'&&ego.cs&&ego.cs.key===c.key&&ego.cs.done&&!ego.cs.counted){ego.cs.counted=true;rules.stopSigns++;logE(`🛑 Full stop at ${c.name}`,'info');}
    if(res.rule!=='free'||nd.turn==='left')break;                       // only the first stop line that can hold the car matters
  }
  // slow down in time for a lower limit ahead
  for(let q=ego.pi;q<Math.min(r.n,ego.pi+120);q+=4){const vl=r.vl[q];if(vl>=v)continue;const dist=Math.max((q-ego.pi)*DS-2,.5);acc=Math.min(acc,(vl*vl-v*v)/(2*dist));}
  rules.driveT+=ego.v>1?dt:0;const over=ego.v-ego.vlim;if(over>.6){rules.overT+=dt;}rules.maxOver=Math.max(rules.maxOver,over*3.6);
  return acc;
}
// controlled intersections a route still has to cross where the taxi may have to stop (feature for the ETA model)
function ctrlCount(r,from=0){return r&&r.nodes?r.nodes.filter(n=>n.ctrl&&n.qs>=from&&approachRule(n.ctrl,axisOf(n.ad))!=='free').length:0;}
// ============================== pedestrians ==============================
// A walker about to cross a street: at a signal it waits for the walk phase (crossing east-west needs the
// east-west green), and everywhere it looks first and doesn't step out in front of a car that is close and moving.
function walkNeed(n,t){if(!t||t.i!==n.i||t.j!==n.j)return null;const c=ctrlAt(n.i,n.j),a=nodePos(n),b=nodePos(t);
  return{c,axis:t.qx!==n.qx?'h':'v',mx:(a.x+b.x)/2,my:(a.y+b.y)/2};}
function walkOK(w){
  if(w.c&&w.c.type==='signal'){const st=sigState(w.c,w.axis);if(st.s!=='G'||st.rem<=WALK_MIN)return false;}
  for(const o of actors){if(!(o.kind==='car'||o.kind==='ego'))continue;const dd=Math.hypot(o.x-w.mx,o.y-w.my);
    if(dd<3||(dd<5.5&&o.v>.3))return false;   // a stopped car at the crossing: walk in front of it
    if(o.v>1&&dd<Math.max(14,o.v*3.2)){const h=o.kind==='ego'?[Math.cos(ego.th),Math.sin(ego.th)]:DIR[o.d];if((w.mx-o.x)*h[0]+(w.my-o.y)*h[1]>-2)return false;}}
  return true;}
// ============================== drawing ==============================
// world point at `back` metres upstream of the centre and `right` metres to the right of an approach heading d
function appPt(c,d,back,right){const hx=horiz(d),s=sgnD(d),rr=rightV(d)[hx?1:0],al=(hx?c.i:c.j)*P-s*back,pp=(hx?c.j:c.i)*P+rr*right;return hx?{x:al,y:pp}:{x:pp,y:al};}
const approaches=c=>[0,1,2,3].filter(d=>inGrid({i:c.i-DIR[d][0],j:c.j-DIR[d][1]}));
// The static markings (no stop line where traffic has right of way, yield teeth, STOP lettering) are
// painted into the aerial image by robotaxi/texture.py in Python.
const SIGCOL={G:'#2bd96b',Y:'#ffbf1f',R:'#ff3b4a'};
// Live world-space overlay: glowing signal bars on the stop lines.
function drawTrafficWorld(c2,ep){
  for(const c of Object.values(CTRL)){if(c.type!=='signal'||Math.abs(c.i*P-ep.x)>95||Math.abs(c.j*P-ep.y)>95)continue;
    for(const d of approaches(c)){const st=sigState(c,axisOf(d)),p=appPt(c,d,STOPL+.22,2);
      c2.save();c2.translate(p.x,p.y);c2.rotate(Math.atan2(DIR[d][1],DIR[d][0]));c2.fillStyle=SIGCOL[st.s];
      c2.globalAlpha=.22;c2.fillRect(-.6,-1.9,1.2,3.8);c2.globalAlpha=.95;c2.fillRect(-.22,-1.7,.44,3.4);c2.restore();}}
  c2.globalAlpha=1;
}
// Screen-space signs (always upright, so they stay readable in heading-up mode).
function drawSigns(c2,ep){
  if(Z<4.5)return;const sz=clamp(Z*1.35,9,20),hd=[Math.cos(ego.th),Math.sin(ego.th)];
  const vis=(x,y)=>{const[sx,sy]=w2s(x,y);return sx>-30&&sy>-30&&sx<cw+30&&sy<ch+30?[sx,sy]:null;};
  for(const c of Object.values(CTRL)){if(Math.abs(c.i*P-ep.x)>80||Math.abs(c.j*P-ep.y)>80)continue;
    for(const d of approaches(c)){const rule=approachRule(c,axisOf(d));if(rule==='free')continue;
      const p=appPt(c,d,STOPL+1.2,RH+1.4),s=vis(p.x,p.y);if(!s)continue;const[sx,sy]=s;
      const facing=DIR[d][0]*hd[0]+DIR[d][1]*hd[1]>.5;c2.globalAlpha=facing?1:.5;
      if(rule==='signal'){const st=sigState(c,axisOf(d)),w=sz*.62,h=sz*1.55;
        c2.fillStyle='#14181d';rr(c2,sx-w/2,sy-h/2,w,h,w*.3);c2.fill();c2.strokeStyle='rgba(255,255,255,.35)';c2.lineWidth=1;c2.stroke();
        ['R','Y','G'].forEach((k,n)=>{const on=st.s===k;c2.fillStyle=on?SIGCOL[k]:'#3a3f45';c2.beginPath();c2.arc(sx,sy-h/2+h*(n+.5)/3,w*.3,0,7);c2.fill();
          if(on){c2.globalAlpha*=.35;c2.beginPath();c2.arc(sx,sy-h/2+h*(n+.5)/3,w*.55,0,7);c2.fill();c2.globalAlpha=facing?1:.5;}});
        if(facing&&Z>7){c2.font=`700 ${Math.round(sz*.55)}px "JetBrains Mono",monospace`;c2.fillStyle='#e6eef5';c2.textAlign='left';c2.fillText(Math.ceil(st.rem),sx+w/2+3,sy);}}
      else if(rule==='stop'){const R=sz*.62;c2.fillStyle='#d9232e';c2.strokeStyle='#fff';c2.lineWidth=1.4;c2.beginPath();
        for(let k=0;k<8;k++){const a=Math.PI/8+k*Math.PI/4;k?c2.lineTo(sx+R*Math.cos(a),sy+R*Math.sin(a)):c2.moveTo(sx+R*Math.cos(a),sy+R*Math.sin(a));}c2.closePath();c2.fill();c2.stroke();
        if(sz>11){c2.fillStyle='#fff';c2.font=`800 ${Math.round(R*.62)}px Inter, Arial`;c2.textAlign='center';c2.fillText('STOP',sx,sy+.5);}}
      else{const R=sz*.7;c2.fillStyle='#fff';c2.strokeStyle='#d9232e';c2.lineWidth=Math.max(2,R*.22);c2.beginPath();c2.moveTo(sx-R,sy-R*.6);c2.lineTo(sx+R,sy-R*.6);c2.lineTo(sx,sy+R*.95);c2.closePath();c2.fill();c2.stroke();}
    }
    // speed limit signs just after the intersection, on the right side of each street leaving it
    for(let d=0;d<4;d++){if(!inGrid({i:c.i+DIR[d][0],j:c.j+DIR[d][1]}))continue;const p=appPt(c,d,-13,RH+1.4),s=vis(p.x,p.y);if(!s)continue;const[sx,sy]=s;
      const lim=limitKmh(d,c.i*P,c.j*P),w=sz*1.05,h=sz*1.35,facing=DIR[d][0]*hd[0]+DIR[d][1]*hd[1]>.5;c2.globalAlpha=facing?1:.5;
      c2.fillStyle='#fff';c2.strokeStyle='#111';c2.lineWidth=1.2;rr(c2,sx-w/2,sy-h/2,w,h,2);c2.fill();c2.stroke();
      c2.fillStyle='#111';c2.textAlign='center';if(sz>12){c2.font=`700 ${Math.max(5,Math.round(sz*.24))}px Inter, Arial`;c2.fillText('MAXIMUM',sx,sy-h*.27);}
      c2.font=`800 ${Math.round(sz*(sz>12?.62:.7))}px Inter, Arial`;c2.fillText(lim,sx,sy+h*(sz>12?.14:0));}
  }
  c2.globalAlpha=1;c2.textAlign='left';
}
function drawTrafficMini(m,ms){
  for(const c of Object.values(CTRL)){const x=(c.i*P+M)*ms,y=(c.j*P+M)*ms;
    if(c.type==='signal'){const h=sigState(c,'h').s,v=sigState(c,'v').s;m.fillStyle=SIGCOL[h];m.fillRect(x-5,y-1.5,10,3);m.fillStyle=SIGCOL[v];m.fillRect(x-1.5,y-5,3,10);}
    else{m.fillStyle=c.type==='yield'?'#ffffff':'#d9232e';m.beginPath();m.arc(x,y,2.6,0,7);m.fill();}}
}
// ============================== panel text ==============================
function ctlText(){
  const k=ego.ctl;if(!k)return'<span class="mute">No controlled intersection ahead</span>';
  const c=k.c,d=Math.max(0,k.g).toFixed(0),rl=k.rule;
  if(rl==='signal'){const st=k.st,nm={G:'Green',Y:'Amber',R:'Red'}[st.s];
    const doing=k.hold?(k.why==='left'?'waiting to turn left':ego.v<.3?'stopped':'stopping'):'proceeding';
    return `<i class="dot" style="background:${SIGCOL[st.s]}"></i><b>${nm}</b> ${Math.ceil(st.rem)} s · ${c.name} · ${d} m · ${doing}`;}
  if(rl==='stop'){const doing=!ego.cs||!ego.cs.done?(ego.v<.3?'full stop':'stopping'):k.hold?(k.why==='queue'?'waiting its turn':'waiting for a gap'):'proceeding';
    return `🛑 <b>${c.type==='allstop'?'All-way stop':'Stop sign'}</b> · ${c.name} · ${d} m · ${doing}`;}
  if(rl==='yield')return `⚠️ <b>Yield</b> · ${c.name} · ${d} m · ${k.hold?'giving way':'clear, proceeding'}`;
  return `<b>Right of way</b> · ${c.name} · ${d} m${k.hold?' · waiting to turn left':''}`;
}
function updateTrafficUI(){
  const lim=Math.round((ego.vlim||VLIM)*3.6);$('hLim').textContent=lim;$('hCtl').innerHTML=ctlText();
  if(!$('rLim'))return;
  $('rLim').textContent=lim+' km/h';$('rSpd').textContent=(ego.v*3.6).toFixed(0)+' km/h';$('rSpd').style.color=ego.v>ego.vlim+.6?'#ff4d5e':'';
  $('rNow').innerHTML=ctlText();
  $('rSig').textContent=Object.values(CTRL).filter(c=>c.type==='signal').length;
  $('rRed').textContent=rules.redStops;$('rStop').textContent=rules.stopSigns;$('rYield').textContent=rules.yields;
  $('rViol').textContent=rules.violations;$('rViol').style.color=rules.violations?'#ff4d5e':'#3ddc84';
  $('rOver').textContent=(rules.driveT?100*rules.overT/rules.driveT:0).toFixed(1)+'%';$('rMax').textContent='+'+Math.max(0,rules.maxOver).toFixed(1)+' km/h';
  const near=Object.values(CTRL).map(c=>({c,dd:Math.hypot(c.i*P-ego.x,c.j*P-ego.y)})).sort((a,b)=>a.dd-b.dd).slice(0,7);
  $('rNear').innerHTML=near.map(({c,dd})=>{let st='';
    if(c.type==='signal'){const h=sigState(c,'h'),v=sigState(c,'v');st=`<span style="color:${SIGCOL[h.s]}">E-W ${h.s} ${Math.ceil(h.rem)}s</span> · <span style="color:${SIGCOL[v.s]}">N-S ${v.s} ${Math.ceil(v.rem)}s</span>`;}
    else if(c.type==='allstop')st=`all stop${c.q.length?' · '+c.q.length+' queued':''}`;
    else st=`${c.approach==='h'?'E-W':'N-S'} ${c.type==='stop'?'stops':'yields'}`;
    return `<tr><td>${c.name.replace(/ St W| Ave| St/g,'')}</td><td>${CTRL_NAME[c.type]}</td><td>${dd.toFixed(0)} m</td><td style="text-align:left">${st}</td></tr>`;}).join('');
  const a=PY.analytics&&PY.analytics.rules,box=$('rPy');
  if(box)box.innerHTML=!PY.online?'Offline mode: start the Python server to get the compliance report.':!a?'Waiting for enough driving data…':
    `pandas checked <b>${a.samples}</b> telemetry rows: over the limit <b>${a.pct_over_limit}%</b> of moving time (worst +${a.max_over_kmh} km/h). Stops: <b>${a.red_stops}</b> at red lights, <b>${a.sign_stops}</b> at stop signs, <b>${a.yield_holds}</b> yields. Mean speed by limit: ${Object.entries(a.speed_by_limit||{}).map(([k,v])=>`${k} zone ${v} km/h`).join(', ')}.`;
}
