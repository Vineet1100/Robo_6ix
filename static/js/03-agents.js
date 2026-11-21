/* =============================================================================
 * 03-agents.js — Traffic: cars, streetcars, pedestrians, animals
 * Spawning, lane following and turning (choosePlan / stepVehicle), IDM car-following (idmAccel / npcAccel), sidewalk walking graph, scripted hazards.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== agents ==============================
let actors=[],nextId=1,simT=0;
let ego;
function mkCar(x,y,d,o={}){return Object.assign({id:nextId++,kind:'car',x,y,d,hd:Math.atan2(DIR[d][1],DIR[d][0]),v:rnd(6,10),acc:0,L:rnd(4.3,5.0),Wd:rnd(1.8,2.0),color:pick(CARCOLS),plan:null,v0:rnd(9,12.5),vf:rnd(.88,1.05),stopT:0,ignoreUntil:0},o);}
function choosePlan(a){
  const s=sgnD(a.d),hx=horiz(a.d),al=hx?a.x:a.y,perp=hx?a.y:a.x;
  const k=s>0?Math.ceil((al+2.6)/P):Math.floor((al-2.6)/P);
  const cross=k*P,rc=Math.round(perp/P)*P;
  const opts=[];
  const straightOK=cross+s*P>=0&&cross+s*P<=W&&cross>=0&&cross<=W;
  if(straightOK)opts.push({d2:a.d,w:.5});
  if(cross>=0&&cross<=W)for(const d2 of [(a.d+1)%4,(a.d+3)%4]){const s2=sgnD(d2);if(rc+s2*P>=0&&rc+s2*P<=W)opts.push({d2,w:.25});}
  if(a.tram){if(straightOK)return{cross,d2:a.d,at:cross+s*3};return{cross,d2:(a.d+2)%4,at:s*(cross-s*P/2-al)>0?cross-s*P/2:al};}   // streetcars turn back mid-block, clear of intersections
  if(!opts.length)return{cross:al,d2:(a.d+2)%4,at:al};
  let ch=opts[0];
  if(a.forceStraightAt!=null&&Math.abs(a.forceStraightAt-cross)<1&&straightOK){ch={d2:a.d};a.forceStraightAt=null;}
  else{let r=Math.random()*opts.reduce((t,o)=>t+o.w,0);for(const o of opts){r-=o.w;if(r<=0){ch=o;break;}}}
  const at=ch.d2===a.d?cross+s*3:cross+LO*rightV(ch.d2)[hx?0:1];
  return{cross,d2:ch.d2,at};
}
function stepVehicle(a,dt){
  if(!a.plan)a.plan=choosePlan(a);
  a.v=Math.max(0,a.v+a.acc*dt);
  const s=sgnD(a.d),hx=horiz(a.d);
  if(hx)a.x+=s*a.v*dt;else a.y+=s*a.v*dt;
  const al=hx?a.x:a.y;
  if(s*(al-a.plan.at)>=0){
    if(a.plan.d2===(a.d+2)%4&&uturnBlocked(a)){if(hx)a.x-=s*a.v*dt;else a.y-=s*a.v*dt;a.v=0;return;}   // wait until the other lane is clear
    if(a.plan.d2!==a.d){if(hx)a.x=a.plan.at;else a.y=a.plan.at;
      if(a.plan.d2===(a.d+2)%4){const rc=Math.round((hx?a.y:a.x)/P)*P;if(hx)a.y=laneCoord(a.plan.d2,rc);else a.x=laneCoord(a.plan.d2,rc);}
      a.d=a.plan.d2;}
    a.plan=null;if(a.noYield&&a.noYieldClear)a.noYield=false;
  }
  const th=Math.atan2(DIR[a.d][1],DIR[a.d][0]);a.hd+=angDiff(a.hd,th)*Math.min(1,dt*7);
}
// A U-turn at the edge of the map swaps lanes in one step, so first make sure nothing is in the way.
function uturnBlocked(a){const d2=(a.d+2)%4,hx=horiz(a.d),rc=Math.round((hx?a.y:a.x)/P)*P,lc=laneCoord(d2,rc),al=hx?a.x:a.y;
  for(const o of actors){if(o===a||!(o.kind==='car'||o.kind==='ego'))continue;const oal=hx?o.x:o.y,operp=hx?o.y:o.x;
    if(Math.abs(operp-lc)<3.6&&Math.abs(oal-al)<a.L/2+(o.L||4.4)/2+5)return true;}
  return false;}
function idmAccel(a,egoMode){
  const h=DIR[a.d],r=rightV(a.d);let gap=1e9,lv=0;
  if(!egoMode&&ego){
    const c=Math.cos(ego.th),s2=Math.sin(ego.th);
    for(const k of [-1.25,0,1.25]){const px=ego.x+c*k-a.x,py=ego.y+s2*k-a.y,al=px*h[0]+py*h[1],lat=px*r[0]+py*r[1];
      if(al>0&&al<45&&Math.abs(lat)<2.05&&!(a.noYield)){const g2=al-a.L/2-1.05;if(g2<gap){gap=g2;lv=Math.max(0,ego.v*(c*h[0]+s2*h[1]));}}}
    if(!a.noYield){const s3=sgnD(a.d),al0=horiz(a.d)?a.x:a.y,cr=s3>0?Math.ceil(al0/P)*P:Math.floor(al0/P)*P,dI=s3*(cr-al0);
      const fr=dI-a.L/2;   // front bumper to the centre (matters for 15 m streetcars)
      if(fr>STOPL-1.5&&fr<20){const perp=horiz(a.d)?a.y:a.x,rc=Math.round(perp/P)*P,cx=horiz(a.d)?cr:rc,cy=horiz(a.d)?rc:cr,de=Math.hypot(ego.x-cx,ego.y-cy);
        const ctl=ctrlAt(Math.round(cx/P),Math.round(cy/P));   // with a traffic control the rules decide; only an ego already in the box is waited for
        const toward=(cx-ego.x)*c+(cy-ego.y)*s2>0;if(de<9||(!ctl&&de<18&&toward&&ego.v>.3)){const g2=fr-STOPL;if(g2<gap){gap=Math.max(g2,.1);lv=0;}}}}
    const gs=npcStopGap(a);if(gs<gap){gap=gs;lv=0;}            // red light, stop sign or yield (13-traffic.js)
  }
  for(const o of actors){if(o===a||(o===ego&&!egoMode))continue;
    const dx=o.x-a.x,dy=o.y-a.y,al=dx*h[0]+dy*h[1];if(al<=0||al>45)continue;
    const lat=dx*r[0]+dy*r[1];let ov=0,ol;
    if(o.kind==='car'||o.kind==='ego'){
      if(o.d===a.d){if(Math.abs(lat)>1.8)continue;ov=o.v;ol=o.L/2;}
      else if((o.d+2)%4===a.d)continue;
      else{if(egoMode||a.noYield)continue;if(o.kind!=='ego'&&simT<a.ignoreUntil)continue;if(Math.abs(lat)>2.6||al>14)continue;ol=1.2;}
    }else{if(egoMode)continue;if(Math.abs(lat)>1.9||al>25)continue;ol=.4;}
    const g2=al-a.L/2-ol;if(g2<gap){gap=g2;lv=ov;}
  }
  const v=a.v,v0=a.v0;let acc=1.6*(1-Math.pow(v/v0,4));
  if(gap<1e8){const ss=2+v*1.3+v*(v-lv)/(2*Math.sqrt(1.6*2.5));acc=1.6*(1-Math.pow(v/v0,4)-Math.pow(Math.max(ss,0)/Math.max(gap,.1),2));}
  if(a.plan&&a.plan.d2!==a.d){const al=horiz(a.d)?a.x:a.y,dist=sgnD(a.d)*(a.plan.at-al);
    if(dist<22){const vt=4.5;if(v>vt)acc=Math.min(acc,(vt*vt-v*v)/(2*Math.max(dist,1)));}}
  return clamp(acc,-9,2);
}
function npcAccel(a,dt){
  if(a.script==='brake'){
    if(simT<a.brakeAt)return 0;
    if(!a.held){if(a.v>0)return -7.5;a.held=simT;}
    if(simT-a.held<4)return -1;
    a.script=null;
  }
  if(!a.fixV0){const lim=limitKmh(a.d,a.x,a.y)/3.6;a.v0=a.tram?Math.min(9,lim):lim*a.vf;}   // each street's speed limit
  const acc=idmAccel(a,false);
  if(a.v<.1&&acc<.1){a.stopT+=dt;if(a.stopT>4){a.ignoreUntil=simT+3;a.stopT=0;}}else a.stopT=0;
  return acc;
}
// walkers on the sidewalk graph
const nodePos=n=>({x:n.i*P+n.qx*NODE,y:n.j*P+n.qy*NODE});
function nodeOK(n){const p=nodePos(n);return n.i>=0&&n.i<=N&&n.j>=0&&n.j<=N&&p.x>=0&&p.x<=W&&p.y>=0&&p.y<=W;}
function neighbors(n){return[{n:{i:n.i,j:n.j,qx:-n.qx,qy:n.qy},w:.13},{n:{i:n.i,j:n.j,qx:n.qx,qy:-n.qy},w:.13},{n:{i:n.i+n.qx,j:n.j,qx:-n.qx,qy:n.qy},w:.37},{n:{i:n.i,j:n.j+n.qy,qx:n.qx,qy:-n.qy},w:.37}].filter(o=>nodeOK(o.n));}
function randNode(){for(;;){const n={i:Math.floor(Math.random()*(N+1)),j:Math.floor(Math.random()*(N+1)),qx:Math.random()<.5?-1:1,qy:Math.random()<.5?-1:1};if(nodeOK(n))return n;}}
const SPD={ped:[1.1,1.6],child:[1.0,1.4],dog:[1.4,2.0],cat:[.6,1.0]};
function mkWalker(kind){const n=randNode(),p=nodePos(n),nb=neighbors(n),t=pick(nb).n,q=nodePos(t),f=Math.random();
  return{id:nextId++,kind,x:p.x+(q.x-p.x)*f,y:p.y+(q.y-p.y)*f,vx:0,vy:0,hd:0,sp:rnd(...SPD[kind]),node:n,target:t,wait:0,mode:'graph',xok:true,col:pick(SHIRTS),fur:pick(['#8b5a2b','#d9c3a0','#2b2b2b','#a0743f','#e8e2d6']),phase:Math.random()*6};}
function stepWalker(a,dt){
  if(a.mode==='script')return stepScript(a,dt);
  if(a.mode==='wait'){a.vx=a.vy=0;return;}
  if(a.mode==='walkTo'){const dx=a.goto.x-a.x,dy=a.goto.y-a.y,dd=Math.hypot(dx,dy),sp=a.sp||1.3;if(dd<.25){a.arrived=true;a.vx=a.vy=0;return;}
    a.arrived=false;const st=Math.min(sp*dt,dd);a.vx=dx/dd*sp;a.vy=dy/dd*sp;a.x+=dx/dd*st;a.y+=dy/dd*st;a.hd=Math.atan2(dy,dx);return;}
  if(a.mode==='graze')return stepGraze(a,dt);
  if(a.wait>0){a.wait-=dt;a.vx=a.vy=0;return;}
  if(!a.target){const nb=neighbors(a.node);let r=Math.random()*nb.reduce((t,o)=>t+o.w,0);let ch=nb[0];for(const o of nb){r-=o.w;if(r<=0){ch=o;break;}}a.target=ch.n;a.xok=false;}
  if(!a.xok){const w=walkNeed(a.node,a.target);if(w&&!walkOK(w)){a.vx=a.vy=0;a.walkWait=true;return;}a.xok=true;a.walkWait=false;}   // wait for the walk signal
  const p=nodePos(a.target),dx=p.x-a.x,dy=p.y-a.y,dd=Math.hypot(dx,dy);
  let sp=a.sp;if(a.kind==='dog')sp*=1+.3*Math.sin(simT*3+a.phase);
  if(dd<sp*dt+1e-6){a.x=p.x;a.y=p.y;a.node=a.target;a.target=null;a.wait=Math.random()<.25?rnd(.5,3):0;a.vx=a.vy=0;return;}
  a.vx=dx/dd*sp;a.vy=dy/dd*sp;a.x+=a.vx*dt;a.y+=a.vy*dt;a.hd=Math.atan2(a.vy,a.vx);
}
function stepGraze(a,dt){
  if(a.wait>0){a.wait-=dt;a.vx=a.vy=0;return;}
  if(!a.gt){a.gt={x:rnd(a.bx0,a.bx1),y:rnd(a.by0,a.by1)};}
  const dx=a.gt.x-a.x,dy=a.gt.y-a.y,dd=Math.hypot(dx,dy),sp=.6;
  if(dd<sp*dt+.05){a.gt=null;a.wait=rnd(2,7);a.vx=a.vy=0;return;}
  a.vx=dx/dd*sp;a.vy=dy/dd*sp;a.x+=a.vx*dt;a.y+=a.vy*dt;a.hd=Math.atan2(a.vy,a.vx);
}
function mkScript(kind,pts,o={}){return Object.assign({id:nextId++,kind,x:pts[0].x,y:pts[0].y,vx:0,vy:0,hd:Math.atan2(pts[1].y-pts[0].y,pts[1].x-pts[0].x),mode:'script',path:pts,pi:1,wait:pts[0].pause||0,born:simT,col:pick(SHIRTS),fur:'#8b5a2b',phase:Math.random()*6,scripted:true},o);}
function stepScript(a,dt){
  if(a.wait>0){a.wait-=dt;a.vx=a.vy=0;return;}
  if(a.pi>=a.path.length){a.vx=a.vy=0;a.done=true;return;}
  const p=a.path[a.pi],dx=p.x-a.x,dy=p.y-a.y,dd=Math.hypot(dx,dy),sp=p.sp;
  if(dd<=sp*dt){a.x=p.x;a.y=p.y;a.pi++;a.wait=p.pause||0;return;}
  a.vx=dx/dd*sp;a.vy=dy/dd*sp;a.x+=a.vx*dt;a.y+=a.vy*dt;a.hd=Math.atan2(a.vy,a.vx);
}

