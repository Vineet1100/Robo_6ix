/* =============================================================================
 * 08-scenarios.js — Crunch-time tests (keys 1–6)
 * Jaywalker, child & ball, raccoon, dog, lead-car hard brake, red-light runner. Add or edit hazards here.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== scenarios ==============================
function frameAt(D){
  const r=route;if(!r)return null;
  let q=Math.min(r.n-1,ego.pi+Math.round(D/DS));
  for(let t=0;t<30&&q<r.n-4;t++,q+=4){
    const tx=r.tx[q],ty=r.ty[q];if(Math.abs(tx)<.995&&Math.abs(ty)<.995)continue;
    const d=Math.abs(tx)>.9?(tx>0?0:2):(ty>0?1:3),hx=horiz(d),px=r.px[q],py=r.py[q],al=hx?px:py,perp=hx?py:px;
    if(Math.abs(al-Math.round(al/P)*P)<8)continue;
    return{hx,s:sgnD(d),al,rc:Math.round(perp/P)*P,rr:rightV(d)[hx?1:0],d,q};
  }
  return null;
}
const PT=(F,al,pp,sp,pause)=>{const o=F.hx?{x:al,y:pp}:{x:pp,y:al};if(sp)o.sp=sp;if(pause)o.pause=pause;return o;};
function clearLane(x,y,r){actors=actors.filter(o=>o===ego||o.kind!=='car'||o.tram||Math.hypot(o.x-x,o.y-y)>r);}
const SCEN=[
  {k:'jay',name:'Jaywalker',icon:'🚶',fn(){const F=frameAt(ego.v*2.4+6);if(!F)return false;const a=F.al;
    const off=rnd(-25,25);
    actors.push(mkScript('ped',[PT(F,a,F.rc+F.rr*5.6),PT(F,a,F.rc-F.rr*5.4,1.5),PT(F,clamp(a+off,2,W-2),F.rc-F.rr*5.4,1.3)]));return a;}},
  {k:'child',name:'Child chasing ball',icon:'⚽',fn(){const F=frameAt(ego.v*3+6);if(!F)return false;const a=F.al;
    actors.push(mkScript('ball',[PT(F,a,F.rc-F.rr*6),PT(F,a+F.s*.5,F.rc+F.rr*9,4.5)]));
    actors.push(mkScript('child',[PT(F,a-F.s*1,F.rc-F.rr*6.5,0,1.0),PT(F,a,F.rc+F.rr*5.8,2.8),PT(F,a+F.s*.5,F.rc+F.rr*8,1)]));return a;}},
  {k:'raccoon',name:'Raccoon freezes in lane',icon:'🦝',fn(){const F=frameAt(ego.v*3+8);if(!F)return false;const a=F.al;
    actors.push(mkScript('raccoon',[PT(F,a,F.rc+F.rr*7),PT(F,a,F.rc+F.rr*2,4.5,1.8),PT(F,a-F.s*2,F.rc-F.rr*9,4.5)]));return a;}},
  {k:'dog',name:'Dog darts across',icon:'🐕',fn(){const F=frameAt(ego.v*2.5+7);if(!F)return false;const a=F.al;
    actors.push(mkScript('dog',[PT(F,a,F.rc-F.rr*5.5),PT(F,a+F.s*1.5,F.rc-F.rr*1.5,4.5),PT(F,a-F.s*1,F.rc+F.rr*1,4,.6),PT(F,a+F.s*2,F.rc+F.rr*2.6,4),PT(F,a,F.rc+F.rr*6,5)]));return a;}},
  {k:'lead',name:'Lead car slams brakes',icon:'🚗',fn(){const F=frameAt(Math.max(ego.v*1.4,9)+4.5);if(!F)return false;
    const lc=laneCoord(F.d,F.rc);const p=PT(F,F.al,lc);clearLane(p.x,p.y,10);
    const c=mkCar(p.x,p.y,F.d,{v:ego.v,script:'brake',brakeAt:simT+.9,color:'#c8c8c8',scripted:true,born:simT});c.plan=choosePlan(c);
    if(c.plan.d2!==c.d){const cr=c.plan.cross;if(cr+F.s*P>=0&&cr+F.s*P<=W)c.plan={cross:cr,d2:c.d,at:cr+F.s*3};}
    actors.push(c);return F.al;}},
  {k:'run',name:'Cross-traffic runs light',icon:'🚙',fn(){const r=route;if(!r)return false;
    const nd=r.nodes.find(n=>(n.q-ego.pi)*DS>14&&(n.q-ego.pi)*DS<58);if(!nd)return false;
    const q0=Math.max(ego.pi,nd.q-16),tx=r.tx[q0],ty=r.ty[q0];if(Math.abs(tx)<.99&&Math.abs(ty)<.99)return false;
    const d=Math.abs(tx)>.9?(tx>0?0:2):(ty>0?1:3),hx=horiz(d);
    const F={hx,s:sgnD(d),rc:hx?nd.j*P:nd.i*P,rr:rightV(d)[hx?1:0]};const cross=hx?nd.i*P:nd.j*P;
    const dist=(nd.q-ego.pi)*DS,tE=dist/Math.max(ego.v,4),vR=11,off=vR*tE;
    let side=1,d2=(d+3)%4;
    if(F.rc+F.rr*off<2||F.rc+F.rr*off>W-2){side=-1;d2=(d+1)%4;if(F.rc-F.rr*off<2||F.rc-F.rr*off>W-2)return false;}
    const lc=laneCoord(d2,cross),pp=F.rc+side*F.rr*off,pos=hx?{x:lc,y:pp}:{x:pp,y:lc};
    clearLane(pos.x,pos.y,12);
    const ct=ctrlAt(nd.i,nd.j);if(ct&&ct.type==='signal'){const ax=axisOf(d);ct.offset=(((ax==='h'?1:24)-simT)%CYCLE+CYCLE)%CYCLE;}   // taxi gets the green, the runner has a red
    const c=mkCar(pos.x,pos.y,d2,{v:vR,v0:12.5,fixV0:true,noYield:true,noYieldClear:true,color:'#b8322c',scripted:true,born:simT});c.forceStraightAt=F.rc;c.plan=choosePlan(c);
    actors.push(c);return cross;}},
];
function runScenario(i){
  const sc=SCEN[i];const r=sc.fn();
  if(r===false||r==null){logE(`Not enough road ahead for "${sc.name}" — try again in a moment`,'info');return false;}
  logE(`TEST ▸ ${sc.name} injected ahead`,'info');return true;
}

