/* =============================================================================
 * 07-planner.js — Perception, prediction, collision risk, decisions
 * Tracks + confidence, predicted paths, P(collision) per object, the six manoeuvres (ACTIONS) and the cost function that picks one.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== perception, prediction, planning ==============================
const HZ=4,PDT=.1,NS=Math.round(HZ/PDT);
const ACTIONS=[
  {k:'CRUISE',label:'Cruise / follow',a:null,lat:0,comfort:0},
  {k:'SLOW',label:'Ease off',a:-1.5,lat:0,comfort:1.2},
  {k:'BRAKE',label:'Firm brake',a:-4,lat:0,comfort:4},
  {k:'EMERGENCY',label:'Emergency brake',a:-8,lat:0,comfort:11},
  {k:'NUDGE_L',label:'Nudge left',a:-1,lat:-1.1,comfort:3.5},
  {k:'NUDGE_R',label:'Nudge right',a:-1,lat:.9,comfort:3},
];
function segDist(p1x,p1y,q1x,q1y,p2x,p2y,q2x,q2y){
  const d1x=q1x-p1x,d1y=q1y-p1y,d2x=q2x-p2x,d2y=q2y-p2y,rx=p1x-p2x,ry=p1y-p2y;
  const a=d1x*d1x+d1y*d1y,e=d2x*d2x+d2y*d2y,f=d2x*rx+d2y*ry;let s,t;
  if(a<=1e-9&&e<=1e-9)return Math.hypot(rx,ry);
  if(a<=1e-9){s=0;t=clamp(f/e,0,1);}
  else{const c=d1x*rx+d1y*ry;
    if(e<=1e-9){t=0;s=clamp(-c/a,0,1);}
    else{const b=d1x*d2x+d1y*d2y,den=a*e-b*b;s=den!==0?clamp((b*f-c*e)/den,0,1):0;t=(b*s+f)/e;
      if(t<0){t=0;s=clamp(-c/a,0,1);}else if(t>1){t=1;s=clamp((b-c)/a,0,1);}}}
  return Math.hypot(p1x+d1x*s-(p2x+d2x*t),p1y+d1y*s-(p2y+d2y*t));
}
function egoPos(){return{x:ego.x,y:ego.y};}
function objModel(o){
  const veh=o.kind==='car';let vx,vy,ax=0,ay=0,hx=0,hy=0,tStop=1e9;
  if(veh){const h=DIR[o.d];hx=h[0];hy=h[1];vx=hx*o.v;vy=hy*o.v;if(o.acc<0){ax=hx*o.acc;ay=hy*o.acc;tStop=o.v/(-o.acc);}}
  else{vx=o.vx;vy=o.vy;}
  const sg=SIG[o.kind],road=onRoad(o.x,o.y);
  const pts=new Float32Array((NS+1)*2);
  for(let i=0;i<=NS;i++){const t=Math.min(i*PDT,tStop);pts[2*i]=o.x+vx*t+.5*ax*t*t;pts[2*i+1]=o.y+vy*t+.5*ay*t*t;}
  const still=!veh&&Math.hypot(vx,vy)<.05&&!road;
  return{o,veh,hl:veh?Math.max(1.4,o.L/2-.95):0,hx,hy,r:SIZE[o.kind].r,s0:sg.s0,k:road?sg.kr:still?.04:sg.ks,pts,road};
}
function egoTraj(act,nominal){
  const pts=new Float32Array((NS+1)*2),hs=new Float32Array((NS+1)*2),r=route;
  const pl=ego.pull||0;let v=ego.v,lat=ego.lat+pl,s=ego.pi*DS,vs=0;const c0=Math.cos(ego.th),s0=Math.sin(ego.th),sStart=s;
  let ox=0,oy=0;
  if(r){const q=ego.pi;ox=ego.x-(r.px[q]-r.ty[q]*lat);oy=ego.y-(r.py[q]+r.tx[q]*lat);}
  for(let i=0;i<=NS;i++){const t=i*PDT;
    if(r){const q=Math.min(r.n-1,Math.round(s/DS)),f=Math.max(0,1-t/1.5);
      pts[2*i]=r.px[q]-r.ty[q]*lat+ox*f;pts[2*i+1]=r.py[q]+r.tx[q]*lat+oy*f;
      if(i===0){hs[0]=c0;hs[1]=s0;}else{hs[2*i]=r.tx[q];hs[2*i+1]=r.ty[q];}}
    else{const dd=s-sStart;pts[2*i]=ego.x+c0*dd;pts[2*i+1]=ego.y+s0*dd;hs[2*i]=c0;hs[2*i+1]=s0;}
    const a=t<.3?ego.acc:Math.max(act.a===null?nominal:Math.min(act.a,nominal),-ctl.mu*9.81*.95);
    v=clamp(v+a*PDT,0,VLIM);s+=v*PDT;vs+=v;
    lat+=clamp(act.lat+pl-lat,-1.2*PDT,1.2*PDT);
  }
  return{pts,hs,vAvg:vs/(NS+1)};
}
function riskOf(m,tr){
  let best=0,ttc=null,d0=0;
  for(let i=0;i<=NS;i++){
    const t=i*PDT,ex=tr.pts[2*i],ey=tr.pts[2*i+1],hx=tr.hs[2*i],hy=tr.hs[2*i+1];
    const ox=m.pts[2*i],oy=m.pts[2*i+1];
    let d;
    if(m.veh)d=segDist(ex-hx*1.2,ey-hy*1.2,ex+hx*1.2,ey+hy*1.2,ox-m.hx*m.hl,oy-m.hy*m.hl,ox+m.hx*m.hl,oy+m.hy*m.hl)-1.0-m.r;
    else d=segDist(ex-hx*1.2,ey-hy*1.2,ex+hx*1.2,ey+hy*1.2,ox,oy,ox,oy)-1.0-m.r;
    if(i===0)d0=d;
    const sig=m.s0+m.k*t,p=d<=0?1:Math.exp(-d*d/(2*sig*sig)),pw=p*(1-.11*t);
    if(pw>best)best=pw;
    if(ttc===null&&d<=.2)ttc=t;
  }
  return{risk:best,ttc,d0};
}
let decision={act:ACTIONS[0],results:[],tracks:[],maxNo:0,crit:null,lat:0,traj:null};
let latHist=[];
function plan(){
  const t0=performance.now();
  const ep=egoPos();
  const tracks=[];
  for(const o of actors){if(o===ego||o.boarding)continue;const d=Math.hypot(o.x-ep.x,o.y-ep.y);if(d>RANGE)continue;
    const tgt=CONF0[o.kind]-.28*Math.pow(d/RANGE,1.6);
    o.conf=o.conf==null?tgt:clamp(o.conf+(tgt+(Math.random()-.5)*.04-o.conf)*.3,.3,.995);
    tracks.push({o,dist:d,m:objModel(o)});}
  const nominal=ego.nominal;
  {const c=Math.cos(ego.th),s=Math.sin(ego.th),evx=c*ego.v,evy=s*ego.v;
   for(const tk of tracks){const o=tk.o,dx=o.x-ep.x,dy=o.y-ep.y;tk.fwd=dx*c+dy*s;tk.lat=-dx*s+dy*c;
     const ovx=o.kind==='car'?DIR[o.d][0]*o.v:o.vx,ovy=o.kind==='car'?DIR[o.d][1]*o.v:o.vy;
     tk.closing=-(dx*(ovx-evx)+dy*(ovy-evy))/Math.max(tk.dist,.1);}}
  const results=ACTIONS.map(act=>{
    const tr=egoTraj(act,nominal);let Rmax=0,Rsum=0;const per=[];
    const mine=ride.req&&ride.phase==='toPickup'?ride.req.rider:null;
    for(const tk of tracks){const r=riskOf(tk.m,tr);per.push(r);if(tk.o===mine&&tk.o.mode==='wait')continue;if(r.risk>Rmax)Rmax=r.risk;Rsum+=r.risk;}
    const cost=160*Rmax+25*Rsum+act.comfort+16*(1-tr.vAvg/VLIM)+(act.lat?2:0);
    return{act,tr,Rmax,Rsum,cost,per};
  });
  let best=results.reduce((a,b)=>b.cost<a.cost?b:a);
  const cur=results.find(r=>r.act===decision.act);
  if(cur&&best!==cur&&cur.Rmax<.25&&best.cost>cur.cost-1.5)best=cur;
  const cruise=results[0];
  tracks.forEach((tk,i)=>{
    const prox=.15*Math.exp(-Math.max(cruise.per[i].d0,0)/2);
    tk.pNo=1-(1-cruise.per[i].risk)*(1-prox);
    tk.pPlan=1-(1-best.per[i].risk)*(1-.15*Math.exp(-Math.max(best.per[i].d0,0)/2));
    tk.ttc=cruise.per[i].ttc;tk.dGap=cruise.per[i].d0;
  });
  tracks.sort((a,b)=>b.pNo-a.pNo||a.dist-b.dist);
  const lat=performance.now()-t0;latHist.push(lat);if(latHist.length>50)latHist.shift();
  const prevK=decision.act.k;
  const crit=tracks.length&&tracks[0].pNo>.35?tracks[0]:null;
  decision={act:best.act,results,tracks,maxNo:tracks.length?tracks[0].pNo:0,crit,lat,traj:best.tr,nominal};
  if(best.act.k!==prevK){
    if(best.act.k==='EMERGENCY')stats.em++;
    if(best.act.k!=='CRUISE'&&best.act.k!=='SLOW'){
      const c=tracks[0];
      logE(`${best.act.label.toUpperCase()} — ${c?CLSNAME[c.o.kind]+' #'+c.o.id+' @ '+c.dist.toFixed(1)+' m, P='+Math.round(c.pNo*100)+'%'+(c.ttc!=null?', TTC '+c.ttc.toFixed(1)+' s':''):''} (${lat.toFixed(2)} ms)`,best.act.k==='EMERGENCY'?'bad':'warn');
    }
  }
}

