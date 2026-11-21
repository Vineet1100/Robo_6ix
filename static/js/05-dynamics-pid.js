/* =============================================================================
 * 05-dynamics-pid.js — Vehicle physics, PID and Ziegler–Nichols
 * Mass, drag, motor power, wet patches and grip, hills, actuator delay, the PID speed controller and the Ziegler–Nichols Ku/Tu calculation and tuning rules.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== vehicle dynamics, PID, Ziegler–Nichols ==============================
const VEH={m0:1850,c:.527,crr:.012,Fmax:5600,Pmax:60000,ta:.25,th:.08,Ts:.02};
const wetZones=[];
(function(){const r=mulberry(77);for(let k=0;k<16;k++){const vert=r()<.5,ri=Math.floor(r()*(N+1)),al=Math.floor(r()*N)*P+12+r()*(P-24),off=(r()-.5)*4,rad=5+r()*7;
  wetZones.push(vert?{x:ri*P+off,y:al,r:rad}:{x:al,y:ri*P+off,r:rad});}})();
function muAt(x,y){for(const z of wetZones)if((x-z.x)**2+(y-z.y)**2<z.r*z.r)return .55;return .9;}
function gradeAt(x,y,th){const gx=3/80*Math.cos(x/80)*Math.cos(y/95)+1.5/50*Math.cos((x+y)/50),gy=-3/95*Math.sin(x/80)*Math.sin(y/95)+1.5/50*Math.cos((x+y)/50);return gx*Math.cos(th)+gy*Math.sin(th);}
const mass=()=>VEH.m0+80*ride.pax;
const Ftr=v=>Math.min(VEH.Fmax,VEH.Pmax/Math.max(v,.5));
const resist=(v,gr,m)=>VEH.c*v*v+(v>.05?VEH.crr*m*9.81:0)+m*9.81*gr;
const RULES={classic:{name:'Classic Ziegler–Nichols',kp:.6,ti:.5,td:.125},pessen:{name:'Pessen integral rule',kp:.7,ti:.4,td:.15},some:{name:'ZN — some overshoot',kp:.33,ti:.5,td:.333},none:{name:'ZN — no overshoot',kp:.2,ti:.5,td:.333}};
function znAt(v,m,mu,mode){
  const v0=Math.max(v,1.5);let b=2*VEH.c*v0;
  if(mode==='traction'&&v0>VEH.Pmax/VEH.Fmax){const u0=clamp(resist(v0,0,m)/Ftr(v0),0,1);b+=VEH.Pmax/(v0*v0)*u0;}
  const Kf=mode==='traction'?Ftr(v0):mu*9.81*m,K=Kf/b,tp=m/b,ta=VEH.ta,th=VEH.th+VEH.Ts/2;
  let lo=1e-3,hi=200;for(let k=0;k<60;k++){const w=(lo+hi)/2;if(Math.atan(w*tp)+Math.atan(w*ta)+w*th<Math.PI)lo=w;else hi=w;}
  const w=(lo+hi)/2,Ku=Math.sqrt(1+(w*tp)**2)*Math.sqrt(1+(w*ta)**2)/K,Tu=2*Math.PI/w;
  return{K,tp,ta,th,Ku,Tu,w,mode,v:v0,m,mu};
}
function ruleGains(z,r){const R=RULES[r],Kp=R.kp*z.Ku,Ti=R.ti*z.Tu,Td=R.td*z.Tu;return{Kp,Ki:Kp/Ti,Kd:Kp*Td,Ti,Td};}
const ctl={vref:0,I:0,dF:0,vPrev:0,u:0,uDel:0,uAct:0,q:[],P:0,D:0,ff:0,e:0,sat:false,clamp:false,g:{Kp:1,Ki:1,Kd:0,Ti:1,Td:0},zn:null,rule:'classic',ffOn:true,znT:-1,hist:[],histT:-1,mu:.9,grade:0,mode:'traction',aCmd:0};
function pidStep(dt,aCmd){
  const m=mass(),mu=muAt(ego.x,ego.y),gr=gradeAt(ego.x,ego.y,ego.th);ctl.mu=mu;ctl.grade=gr;
  aCmd=Math.max(aCmd,-mu*9.81*.95);ctl.aCmd=aCmd;
  ctl.vref=clamp(ctl.vref+aCmd*dt,Math.max(0,ego.v-2.5),Math.min(VLIM,ego.v+2.5));
  const Freq=m*aCmd+resist(ego.v,gr,m);let ff=0;
  if(ctl.ffOn)ff=Freq>=0?Freq/Ftr(ego.v):Freq/(mu*9.81*m);
  ctl.mode=Freq>=0?'traction':'brake';
  if(simT-ctl.znT>=.2||!ctl.zn){ctl.znT=simT;ctl.zn=znAt(ego.v,m,mu,ctl.mode);ctl.g=ruleGains(ctl.zn,ctl.rule);}
  const{Kp,Ki,Kd}=ctl.g,e=ctl.vref-ego.v;
  const dv=(ego.v-ctl.vPrev)/dt;ctl.vPrev=ego.v;const Tf=Math.max(.03,ctl.g.Td/8);ctl.dF+=(dv-ctl.dF)*Math.min(1,dt/Tf);
  const P=Kp*e,D=-Kd*ctl.dF,uRaw=ff+P+ctl.I+D,uSat=clamp(uRaw,-1,1);
  ctl.sat=uSat!==uRaw;ctl.clamp=ctl.sat&&Math.sign(e)===Math.sign(uRaw);
  if(!ctl.clamp)ctl.I=clamp(ctl.I+Ki*e*dt,-.6,.6);
  let uc=uSat;if(ctl.vref<.05&&ego.v<.2){uc=Math.min(uc,-.35);ctl.I*=.97;}
  ctl.u=uc;ctl.P=P;ctl.D=D;ctl.ff=ff;ctl.e=e;
  ctl.q.push([simT,uc]);while(ctl.q.length&&ctl.q[0][0]<=simT-VEH.th)ctl.uDel=ctl.q.shift()[1];
  ctl.uAct+=(ctl.uDel-ctl.uAct)*Math.min(1,dt/VEH.ta);
  const ua=ctl.uAct,F=ua>=0?ua*Ftr(ego.v):ua*mu*9.81*m;
  let a=(F-resist(ego.v,gr,m))/m;
  if(ego.v<.05&&ua<=0){const net=-resist(0,gr,m);a=net>-F?(net+F)/m:0;if(a<=0){a=0;ego.v=0;}}   // brakes on at a standstill: the car is held, no creeping
  ego.v=clamp(ego.v+a*dt,0,VLIM+.5);ego.acc=a;
  if(simT-ctl.histT>=.05){ctl.histT=simT;ctl.hist.push([simT,ctl.vref,ego.v,uc,P,ctl.I,D,ff]);if(ctl.hist.length>400)ctl.hist.shift();}
}
const znLog=[];let znLogS=-99;
function znLogStep(){if(odo.total-znLogS<5)return;znLogS=odo.total;
  const z=znAt(ego.v,mass(),ctl.mu,ctl.mode),g=ruleGains(z,ctl.rule);
  znLog.push({t:simT,x:ego.x,y:ego.y,v:ego.v,mu:ctl.mu,gr:ctl.grade,mode:ctl.mode,m:mass(),K:z.K,tau:z.tp,Ku:z.Ku,Tu:z.Tu,Kp:g.Kp,Ki:g.Ki,Kd:g.Kd});
  OUT.zn_log.push({t:+simT.toFixed(2),x:+ego.x.toFixed(2),y:+ego.y.toFixed(2),v:+ego.v.toFixed(2),mu:ctl.mu,grade:+ctl.grade.toFixed(4),mode:ctl.mode,mass:mass(),K:+z.K.toFixed(2),tau:+z.tp.toFixed(2),Ku:+z.Ku.toFixed(4),Tu:+z.Tu.toFixed(4),rule:ctl.rule,Kp:+g.Kp.toFixed(4),Ki:+g.Ki.toFixed(4),Kd:+g.Kd.toFixed(5)});if(znLog.length>3000)znLog.shift();}
function buildZnProfile(r){const S=[],kt=[],kb=[],mu=[],m=mass();
  for(let q=0;q<r.n;q+=10){let km=0;for(let j=Math.max(0,q-20);j<Math.min(r.n,q+20);j++)km=Math.max(km,Math.abs(r.k[j]));
    const vl=r.vl?r.vl[q]:VLIM;let v=Math.min(vl,km>.03?Math.sqrt(2.2/km):vl);v=Math.min(v,Math.sqrt(2*1.5*Math.max((r.n-1-q)*DS,0))+.5);
    const u=muAt(r.px[q],r.py[q]);S.push(q*DS);mu.push(u);kt.push(znAt(v,m,u,'traction').Ku);kb.push(znAt(v,m,u,'brake').Ku);}
  r.zn={s:S,kt,kb,mu,src:'JavaScript'};
  // Ask Python (NumPy, vectorised over the whole route) for the same profile; replace the local one when it arrives.
  if(PY.online){const k=[];for(let q=0;q<r.n;q+=10){let km=0;for(let j=Math.max(0,q-20);j<Math.min(r.n,q+20);j++)km=Math.max(km,Math.abs(r.k[j]));k.push(+km.toFixed(4));}
    api('/api/zn_profile',{s:S,k,mu,mass:mass(),vlim:S.map((_,n)=>r.vl?+r.vl[n*10].toFixed(2):VLIM)}).then(z=>{r.zn={s:z.s,kt:z.kt,kb:z.kb,mu:z.mu,src:'Python · NumPy'};}).catch(()=>{});}}

