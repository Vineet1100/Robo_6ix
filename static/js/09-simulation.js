/* =============================================================================
 * 09-simulation.js — World setup and the main step
 * How many cars / people / animals spawn, the per-tick update order, collision counting, auto-spawned hazards.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== simulation ==============================
const stats={dist:0,em:0,col:0,trips:0};
let paused=false,simSpeed=1,slowF=1,autoChaos=true,autoSlow=true,nextChaos=8;
let planTimer=0;
function initWorld(typeGrid){
  ego={id:0,kind:'ego',x:20,y:laneCoord(0,2*P),th:0,d:0,hd:0,v:8,acc:0,lat:0,steer:0,omega:0,L:4.4,Wd:2.05,v0:VLIM,pi:0,pp:null,cte:0,headErr:0,nominal:0,pull:0};
  actors=[ego];
  let n=0;
  while(n<22){const vert=Math.random()<.5,r=Math.floor(Math.random()*(N+1)),seg=Math.floor(Math.random()*N),al=seg*P+rnd(10,P-10);
    const d=vert?(Math.random()<.5?1:3):(Math.random()<.5?0:2);const lc=laneCoord(d,r*P);
    const x=vert?lc:al,y=vert?al:lc;if(Math.hypot(x-ego.x,y-ego.y)<35)continue;
    if(actors.some(o=>o.kind==='car'&&Math.hypot(o.x-x,o.y-y)<12))continue;
    actors.push(mkCar(x,y,d));n++;}
  for(const[h,r,cnt]of [[true,3,3],[true,0,2],[false,0,2]])for(let k=0;k<cnt;k++){const d=h?(k%2?2:0):(k%2?3:1),al=rnd(20,W-20),lc=laneCoord(d,r*P);
    const x=h?al:lc,y=h?lc:al;if(Math.hypot(x-ego.x,y-ego.y)<40)continue;
    actors.push(mkCar(x,y,d,{tram:true,vf:1,L:15,Wd:2.55,v0:9,v:6,color:'#da291c'}));}
  for(let k=0;k<46;k++)actors.push(mkWalker('ped'));
  for(let k=0;k<5;k++)actors.push(mkWalker('child'));
  for(let k=0;k<8;k++)actors.push(mkWalker('dog'));
  for(let k=0;k<4;k++)actors.push(mkWalker('cat'));
  typeGrid.forEach((t,idx)=>{if(t!=='park')return;const bi=idx%N,bj=Math.floor(idx/N);const x0=bi*P+SW+4,y0=bj*P+SW+4,x1=(bi+1)*P-SW-4,y1=(bj+1)*P-SW-4;
    for(let k=0;k<3;k++)actors.push({id:nextId++,kind:'raccoon',x:rnd(x0,x1),y:rnd(y0,y1),vx:0,vy:0,hd:rnd(0,6),mode:'graze',bx0:x0,bx1:x1,by0:y0,by1:y1,wait:rnd(0,4),fur:'#9a6a3f',phase:0});});
}
let teleT=-1;
function recordTelemetry(){   // sampled every 0.2 s and streamed to Python (pandas) for analysis
  if(simT-teleT<.2)return;teleT=simT;const c=decision.crit;
  OUT.telemetry.push({t:+simT.toFixed(2),x:+ego.x.toFixed(2),y:+ego.y.toFixed(2),v:+ego.v.toFixed(3),vref:+ctl.vref.toFixed(3),u:+ctl.u.toFixed(3),P:+ctl.P.toFixed(3),I:+ctl.I.toFixed(3),
    D:+ctl.D.toFixed(3),ff:+ctl.ff.toFixed(3),action:decision.act.k,max_risk:+decision.maxNo.toFixed(3),crit:c?CLSNAME[c.o.kind]:'',mu:ctl.mu,grade:+ctl.grade.toFixed(4),
    odo_raw:+odo.err.raw.toFixed(3),odo_fus:+odo.err.fus.toFixed(3),vlim:Math.round((ego.vlim||VLIM)*3.6),ctl:ego.ctl?ego.ctl.rule:'',ctl_key:ego.ctl?ego.ctl.c.key:'',sig:ego.ctl&&ego.ctl.st?ego.ctl.st.s:'',hold:ego.ctl&&ego.ctl.hold?ego.ctl.why:'',cte:+(ego.cte||0).toFixed(3),plan_ms:+decision.lat.toFixed(3),phase:ride.phase});}
function step(dt){
  simT+=dt;
  planTimer-=dt;if(planTimer<=0){plan();planTimer=.1;}
  const act=decision.act;
  ego.ruleAcc=egoRules(dt,ego.v);
  const nom=egoNominal();ego.nominal=nom;
  const aCmd=act.a===null?nom:Math.min(act.a,nom);
  ego.lat+=clamp(act.lat-ego.lat,-1.2*dt,1.2*dt);
  rideStep(dt);
  pidStep(dt,aCmd);ppUpdate(dt);egoKinematics(dt);odoStep(dt);znLogStep();recordTelemetry();
  for(const a of actors){
    if(a===ego)continue;
    if(a.kind==='car'){a.acc=npcAccel(a,dt);stepVehicle(a,dt);}
    else stepWalker(a,dt);
  }
  stats.dist+=ego.v*dt;
  const ep=egoPos(),h=[Math.cos(ego.th),Math.sin(ego.th)];
  for(const o of actors){if(o===ego||o.boarding)continue;if(Math.abs(o.x-ep.x)>6||Math.abs(o.y-ep.y)>6)continue;
    let d;if(o.kind==='car'){const oh=DIR[o.d];d=segDist(ep.x-h[0]*1.2,ep.y-h[1]*1.2,ep.x+h[0]*1.2,ep.y+h[1]*1.2,o.x-oh[0]*Math.max(1.4,o.L/2-.95),o.y-oh[1]*Math.max(1.4,o.L/2-.95),o.x+oh[0]*Math.max(1.4,o.L/2-.95),o.y+oh[1]*Math.max(1.4,o.L/2-.95))-1-.95;}
    else d=segDist(ep.x-h[0]*1.2,ep.y-h[1]*1.2,ep.x+h[0]*1.2,ep.y+h[1]*1.2,o.x,o.y,o.x,o.y)-1-SIZE[o.kind].r;
    if(d<=0&&(!o.hitT||simT-o.hitT>4)){o.hitT=simT;stats.col++;logE(`CONTACT with ${CLSNAME[o.kind]} #${o.id} at ${(ego.v*3.6).toFixed(0)} km/h`,'bad');}}
  actors=actors.filter(o=>{if(!o.scripted)return true;const dd=Math.hypot(o.x-ep.x,o.y-ep.y);return !((simT-o.born>40&&dd>40)||(o.done&&dd>45)||simT-o.born>120);});
  if(autoChaos&&simT>nextChaos){if(ego.v>4){const ok=runScenario(Math.floor(Math.random()*SCEN.length));nextChaos=simT+(ok?rnd(9,15):2);}else nextChaos=simT+2;}
}

