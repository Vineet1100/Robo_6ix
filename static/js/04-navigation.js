/* =============================================================================
 * 04-navigation.js — Street names, routing, pure pursuit, odometry
 * HNAME / VNAME street names, Dijkstra route planning, path building with curved turns, pure-pursuit steering, kinematic bicycle model, odometry (dead reckoning + fused fixes).
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== routing, pure pursuit, odometry ==============================
const LW=2.8,DS=.5;
const HNAME=['Queen St W','Richmond St W','Adelaide St W','King St W','Wellington St W','Front St W']; // north → south
const VNAME=['Spadina Ave','John St','University Ave','York St','Bay St','Yonge St']; // west → east
const DNAME=['eastbound','southbound','westbound','northbound'];
let route=null;
const trip={state:'enroute',t0:0,len:0,until:0};
const inGrid=n=>n.i>=0&&n.i<=N&&n.j>=0&&n.j<=N;
function nodeAhead(x,y,d){const hx=horiz(d),s=sgnD(d),al=hx?x:y,perp=hx?y:x;
  const k=s>0?Math.ceil((al+3)/P):Math.floor((al-3)/P),r=Math.round(perp/P);return hx?{i:k,j:r}:{i:r,j:k};}
function destPoint(dest){const A=dest.a,dd=dest.dd,hx=horiz(dd);const al=(hx?A.i*P:A.j*P)+sgnD(dd)*P*dest.frac,lc=laneCoord(dd,hx?A.j*P:A.i*P);return hx?[al,lc]:[lc,al];}
function destName(dest){const A=dest.a,dd=dest.dd;
  if(horiz(dd)){const i2=A.i+DIR[dd][0];return `${HNAME[A.j]}, between ${VNAME[Math.min(A.i,i2)]} & ${VNAME[Math.max(A.i,i2)]}`;}
  const j2=A.j+DIR[dd][1];return `${VNAME[A.i]}, between ${HNAME[Math.min(A.j,j2)]} & ${HNAME[Math.max(A.j,j2)]}`;}
function planRoute(x,y,d,dest){
  const start=nodeAhead(x,y,d);if(!inGrid(start))return null;
  const prevN={i:start.i-DIR[d][0],j:start.j-DIR[d][1]};
  const hx=horiz(d),al=hx?x:y,[dx,dy]=destPoint(dest),dal=hx?dx:dy;
  if(prevN.i===dest.a.i&&prevN.j===dest.a.j&&dest.dd===d&&sgnD(d)*(dal-al)>3)return buildPath(x,y,d,[],dest);
  const key=(i,j,dn)=>(i*(N+1)+j)*4+dn;
  const dist=new Map(),prev=new Map(),pq=[[0,start.i,start.j,d]];dist.set(key(start.i,start.j,d),0);
  let goal=null;
  while(pq.length){pq.sort((a,b)=>a[0]-b[0]);const[c,i,j,din]=pq.shift();const k=key(i,j,din);if(c>dist.get(k))continue;
    if(i===dest.a.i&&j===dest.a.j&&dest.dd!==(din+2)%4&&inGrid({i:i+DIR[dest.dd][0],j:j+DIR[dest.dd][1]})){goal=k;break;}
    for(let dout=0;dout<4;dout++){if(dout===(din+2)%4)continue;const ni=i+DIR[dout][0],nj=j+DIR[dout][1];if(!inGrid({i:ni,j:nj}))continue;
      const nk=key(ni,nj,dout),nc=c+P+(dout===din?0:dout===(din+1)%4?4:12);
      if(!dist.has(nk)||nc<dist.get(nk)){dist.set(nk,nc);prev.set(nk,k);pq.push([nc,ni,nj,dout]);}}}
  if(goal==null)return null;
  const seq=[];let k=goal;while(k!==undefined){const dn=k%4,ij=(k-dn)/4;seq.unshift({i:Math.floor(ij/(N+1)),j:ij%(N+1),din:dn});k=prev.get(k);}
  return buildPath(x,y,d,seq,dest);
}
function buildPath(x,y,d,seq,dest){
  const hx=horiz(d),lc=laneCoord(d,Math.round((hx?y:x)/P)*P);
  const ctrl=[hx?[x,lc]:[lc,y]],turnAt=[];
  for(let k=0;k<seq.length;k++){const n=seq[k],din=n.din,dout=k+1<seq.length?seq[k+1].din:dest.dd;
    if(dout===din){turnAt.push({n,ci:-1,turn:null});continue;}
    let cx,cy;if(horiz(din)){cy=laneCoord(din,n.j*P);cx=laneCoord(dout,n.i*P);}else{cx=laneCoord(din,n.i*P);cy=laneCoord(dout,n.j*P);}
    const right=dout===(din+1)%4,R=right?4.8:7.5;
    const b1=[cx-DIR[din][0]*R,cy-DIR[din][1]*R],b2=[cx+DIR[dout][0]*R,cy+DIR[dout][1]*R];
    ctrl.push(b1);for(let t=1;t<12;t++){const u=t/12;ctrl.push([(1-u)*(1-u)*b1[0]+2*(1-u)*u*cx+u*u*b2[0],(1-u)*(1-u)*b1[1]+2*(1-u)*u*cy+u*u*b2[1]]);}ctrl.push(b2);
    turnAt.push({n,turn:right?'right':'left',onto:horiz(dout)?HNAME[n.j]:VNAME[n.i]});}
  ctrl.push(destPoint(dest));
  const cum=[0];for(let k=1;k<ctrl.length;k++)cum.push(cum[k-1]+Math.hypot(ctrl[k][0]-ctrl[k-1][0],ctrl[k][1]-ctrl[k-1][1]));
  const L=cum[cum.length-1],n=Math.max(3,Math.floor(L/DS)+1);
  const px=new Float32Array(n),py=new Float32Array(n),tx=new Float32Array(n),ty=new Float32Array(n),kk=new Float32Array(n);
  let sg=0;
  for(let q=0;q<n;q++){const s=Math.min(q*DS,L);while(sg<ctrl.length-2&&cum[sg+1]<s)sg++;const sl=Math.max(cum[sg+1]-cum[sg],1e-6),u=clamp((s-cum[sg])/sl,0,1);
    px[q]=ctrl[sg][0]+(ctrl[sg+1][0]-ctrl[sg][0])*u;py[q]=ctrl[sg][1]+(ctrl[sg+1][1]-ctrl[sg][1])*u;}
  for(let q=0;q<n;q++){const a=Math.max(0,q-1),b=Math.min(n-1,q+1);let ax=px[b]-px[a],ay=py[b]-py[a];const l=Math.hypot(ax,ay)||1;tx[q]=ax/l;ty[q]=ay/l;}
  for(let q=1;q<n-1;q++)kk[q]=angDiff(Math.atan2(ty[q-1],tx[q-1]),Math.atan2(ty[q+1],tx[q+1]))/(2*DS);
  const nodes=turnAt.map(t=>{let bq=0,bd=1e9;for(let q=0;q<n;q+=2){const e=(px[q]-t.n.i*P)**2+(py[q]-t.n.j*P)**2;if(e<bd){bd=e;bq=q;}}return{i:t.n.i,j:t.n.j,q:bq,turn:t.turn,onto:t.onto};});
  const R={px,py,tx,ty,k:kk,n,len:L,nodes,dest,name:destName(dest)};annotateRoute(R);return R;   // speed limits + stop lines (13-traffic.js)
}
function randomDest(){const A={i:Math.floor(Math.random()*(N+1)),j:Math.floor(Math.random()*(N+1))};const ds=[0,1,2,3].filter(dd=>inGrid({i:A.i+DIR[dd][0],j:A.j+DIR[dd][1]}));return{a:A,dd:pick(ds),frac:rnd(.3,.7)};}
function setRoute(r,why,kind){
  r.kind=kind||'Destination';route=r;ego.pi=0;trip.state='enroute';trip.t0=simT;trip.len=r.len;odoReset();buildZnProfile(r);
  const turns=r.nodes.filter(n=>n.turn).length;
  logE(`${why} → ${r.name} · route ${r.len.toFixed(0)} m, ${turns} turn${turns===1?'':'s'}`,'info');
}
function newRide(){for(let t=0;t<40;t++){const r=planRoute(ego.x,ego.y,ego.d,randomDest());if(r&&r.len>160&&r.len<520){setRoute(r,'New ride');return;}}
  const r=planRoute(ego.x,ego.y,ego.d,randomDest());if(r)setRoute(r,'New ride');}
function replaceRoute(r){r.kind=route?route.kind:'Pickup';route=r;ego.pi=0;buildZnProfile(r);}
function destFromPoint(x,y){
  const vi=Math.round(x/P),hj=Math.round(y/P),dv=Math.abs(x-vi*P),dh=Math.abs(y-hj*P);let dest;
  if(dh<=dv){const k=clamp(Math.floor(x/P),0,N-1),f=clamp((x-k*P)/P,.2,.8),j=clamp(hj,0,N);
    dest=y>=j*P?{a:{i:k,j},dd:0,frac:f}:{a:{i:k+1,j},dd:2,frac:1-f};}
  else{const k=clamp(Math.floor(y/P),0,N-1),f=clamp((y-k*P)/P,.2,.8),i=clamp(vi,0,N);
    dest=x<=i*P?{a:{i,j:k},dd:1,frac:f}:{a:{i,j:k+1},dd:3,frac:1-f};}
  return dest;
}
function ppUpdate(dt){
  const r=route;if(!r)return;
  let best=ego.pi,bd=1e9;const lo=Math.max(0,best-10),hi=Math.min(r.n-1,best+60);
  for(let q=lo;q<=hi;q++){const e=(r.px[q]-ego.x)**2+(r.py[q]-ego.y)**2;if(e<bd){bd=e;best=q;}}
  ego.pi=best;
  ego.cte=(ego.x-r.px[best])*(-r.ty[best])+(ego.y-r.py[best])*r.tx[best];
  ego.headErr=angDiff(Math.atan2(r.ty[best],r.tx[best]),ego.th);
  const Ld=clamp(2.5+.5*ego.v,3,10),c=Math.cos(ego.th),s=Math.sin(ego.th),rx=ego.x-c*LW/2,ry=ego.y-s*LW/2,lat=ego.lat+(ego.pull||0);
  let q=best,tx=0,ty=0;
  for(;q<r.n;q++){tx=r.px[q]-r.ty[q]*lat;ty=r.py[q]+r.tx[q]*lat;if(Math.hypot(tx-rx,ty-ry)>=Ld)break;}
  if(q>=r.n){q=r.n-1;tx=r.px[q]-r.ty[q]*lat;ty=r.py[q]+r.tx[q]*lat;}
  const dx=tx-rx,dy=ty-ry,lx=dx*c+dy*s,ly=-dx*s+dy*c,ld=Math.max(Math.hypot(lx,ly),.5);
  const kap=2*ly/(ld*ld),tgt=clamp(Math.atan(LW*kap),-.6,.6);
  ego.steer+=clamp(tgt-ego.steer,-1.6*dt,1.6*dt);
  ego.pp={Ld:ld,alpha:Math.atan2(ly,lx),kappa:kap,lx:tx,ly:ty,rx,ry};
}
function egoKinematics(dt){
  let c=Math.cos(ego.th),s=Math.sin(ego.th),rx=ego.x-c*LW/2,ry=ego.y-s*LW/2;
  rx+=ego.v*c*dt;ry+=ego.v*s*dt;
  ego.omega=ego.v/LW*Math.tan(ego.steer);ego.th+=ego.omega*dt;
  if(ego.th>Math.PI)ego.th-=2*Math.PI;if(ego.th<-Math.PI)ego.th+=2*Math.PI;
  ego.x=rx+Math.cos(ego.th)*LW/2;ego.y=ry+Math.sin(ego.th)*LW/2;
  ego.hd=ego.th;ego.d=((Math.round(ego.th/(Math.PI/2))%4)+4)%4;
}
function egoNominal(){
  const r=route,v=ego.v,vd=ego.vlim||VLIM;let acc=1.6*(1-Math.pow(v/vd,4));
  if(!r)return-2;
  const i0=ego.pi;let gap=1e9,lv=0;
  for(const o of actors){if(o.kind!=='car')continue;if(Math.abs(o.x-ego.x)>50||Math.abs(o.y-ego.y)>50)continue;
    let bq=-1,bd=1e9;for(let q=i0;q<Math.min(r.n,i0+100);q+=2){const e=(r.px[q]-o.x)**2+(r.py[q]-o.y)**2;if(e<bd){bd=e;bq=q;}}
    if(bq<0||bd>1.8*1.8)continue;const oh=DIR[o.d],dot=oh[0]*r.tx[bq]+oh[1]*r.ty[bq];if(dot<.5)continue;
    const g=(bq-i0)*DS-ego.L/2-o.L/2;if(g<gap){gap=g;lv=o.v*dot;}}
  if(gap<1e8){const ss=2+v*1.3+v*(v-lv)/(2*Math.sqrt(1.6*2.5));acc=1.6*(1-Math.pow(v/vd,4)-Math.pow(Math.max(ss,0)/Math.max(gap,.1),2));}
  if(ego.ruleAcc!=null)acc=Math.min(acc,ego.ruleAcc);       // traffic lights, stop / yield signs, lower limit ahead
  for(let q=i0;q<Math.min(r.n,i0+70);q+=2){const k=Math.abs(r.k[q]);if(k<.03)continue;const vc=Math.sqrt(2.2/k),dist=Math.max((q-i0)*DS-1,.5);if(v>vc)acc=Math.min(acc,(vc*vc-v*v)/(2*dist));}
  const rem=(r.n-1-i0)*DS;
  if(rem<v*v/(2*1.5)+10){const vt=Math.sqrt(Math.max(0,2*1.5*(rem-.4)));acc=Math.min(acc,clamp((vt-v)*1.8,-6,1.2));}
  return clamp(acc,-6,2);
}
const gauss=()=>{let u=0,v=0;while(!u)u=Math.random();while(!v)v=Math.random();return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);};
const odo={total:0,trip:0,raw:{x:0,y:0,th:0},fus:{x:0,y:0,th:0},bias:0,scale:1,fixT:-9,lastTrail:0,trail:{tru:[],raw:[],fus:[]},start:{x:0,y:0,th:0},err:{raw:0,fus:0}};
function odoReset(){
  odo.raw={x:ego.x,y:ego.y,th:ego.th};odo.fus={x:ego.x,y:ego.y,th:ego.th};odo.start={x:ego.x,y:ego.y,th:ego.th};
  odo.trip=0;odo.lastTrail=0;odo.trail={tru:[],raw:[],fus:[]};
  odo.bias=rnd(.0025,.005)*(Math.random()<.5?-1:1);odo.scale=1+rnd(.01,.015)*(Math.random()<.5?-1:1);
}
function odoStep(dt){
  const vm=Math.max(0,ego.v*odo.scale+(ego.v>.05?gauss()*.04:0)),wm=(ego.omega||0)+odo.bias+gauss()*.008;
  for(const p of [odo.raw,odo.fus]){p.th+=wm*dt;p.x+=vm*Math.cos(p.th)*dt;p.y+=vm*Math.sin(p.th)*dt;}
  odo.total+=vm*dt;odo.trip+=vm*dt;
  if(simT-odo.fixT>1){odo.fixT=simT;const fx=ego.x+gauss()*.25,fy=ego.y+gauss()*.25,ft=ego.th+gauss()*.01;
    odo.fus.x+=.55*(fx-odo.fus.x);odo.fus.y+=.55*(fy-odo.fus.y);odo.fus.th+=.55*angDiff(odo.fus.th,ft);}
  odo.err.raw=Math.hypot(odo.raw.x-ego.x,odo.raw.y-ego.y);odo.err.fus=Math.hypot(odo.fus.x-ego.x,odo.fus.y-ego.y);
  if(odo.trip-odo.lastTrail>.8){odo.lastTrail=odo.trip;const T=odo.trail;T.tru.push(ego.x,ego.y);T.raw.push(odo.raw.x,odo.raw.y);T.fus.push(odo.fus.x,odo.fus.y);
    if(T.tru.length>1600){T.tru.splice(0,2);T.raw.splice(0,2);T.fus.splice(0,2);}}
}

