/* =============================================================================
 * 06-ride-hailing.js — Ride requests and finding the rider
 * Rider requests, GPS error, BLE/UWB radio model, particle filter, pull-over, boarding and drop-off state machine.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== ride-hailing: GPS + RF rider localization ==============================
const RF={P0:-41,n:2.3,sh:3.5,range:85,uwb:35,aoa:8*Math.PI/180};
const rssiAt=d=>RF.P0-10*RF.n*Math.log10(Math.max(d,.5));           // ground truth used to simulate the phone's signal
// The taxi's *model* of the phone's radio. It starts as a generic BLE default (deliberately not the truth) and is
// replaced by the scikit-learn LinearRegression fit computed in Python from logged (distance, RSSI) samples.
const RF_MODEL={P0:-45,n:2.0,src:'generic BLE default'};
const rssiModel=d=>RF_MODEL.P0-10*RF_MODEL.n*Math.log10(Math.max(d,.5));
const ride={phase:'idle',req:null,pax:0,pf:null,est:null,meas:null,lastMeas:0,lastReplan:0,idleT:0,queue:[],nextId:1001,stopErr:[],gpsErr:[],last:null,auto:true,dest:null,tB:0};
function snapSidewalk(x,y){const vi=Math.round(x/P),hj=Math.round(y/P),dv=Math.abs(x-vi*P),dh=Math.abs(y-hj*P);
  if(dh<=dv)y=hj*P+(y>=hj*P?1:-1)*NODE;else x=vi*P+(x>=vi*P?1:-1)*NODE;return[clamp(x,-NODE,W+NODE),clamp(y,-NODE,W+NODE)];}
function logLeg(){if(route)OUT.trips.push({t:+simT.toFixed(1),kind:route.kind,length:+trip.len.toFixed(1),turns:route.nodes.filter(n=>n.turn).length,controls:ctrlCount(route),duration:+(simT-trip.t0).toFixed(1)});}
function makeRequest(x,y,opts){opts=opts||{};
  let p=null;
  if(x!=null){const vi=Math.round(x/P),hj=Math.round(y/P),dv=Math.abs(x-vi*P),dh=Math.abs(y-hj*P);
    if(dh<=dv){const k=clamp(Math.floor(x/P),0,N-1);p={x:clamp(x,k*P+9,k*P+P-9),y:clamp(hj,0,N)*P+(y>=hj*P?1:-1)*NODE};}
    else{const k=clamp(Math.floor(y/P),0,N-1);p={x:clamp(vi,0,N)*P+(x>=vi*P?1:-1)*NODE,y:clamp(y,k*P+9,k*P+P-9)};}
    if(p.x<0||p.x>W||p.y<0||p.y>W){logE('That spot is outside the service area — pick an inner street','warn');return;}}
  else{for(let t=0;t<80;t++){const n=randNode(),nb=neighbors(n).filter(o=>o.n.i!==n.i||o.n.j!==n.j);if(!nb.length)continue;
      const a=nodePos(n),b=nodePos(pick(nb).n),f=rnd(.3,.7),c={x:a.x+(b.x-a.x)*f,y:a.y+(b.y-a.y)*f},dd=Math.hypot(c.x-ego.x,c.y-ego.y);
      if(dd>70&&dd<230){p=c;break;}}
    if(!p)return;}
  const rider={id:nextId++,kind:'ped',x:p.x,y:p.y,vx:0,vy:0,hd:Math.random()*6,mode:'wait',col:opts.user?'#38e1c6':'#b388ff',phase:Math.random()*6,isRider:true,user:!!opts.user};
  actors.push(rider);
  const gps={x:p.x+gauss()*8,y:p.y+gauss()*8},gd=destFromPoint(gps.x,gps.y),gp=destPoint(gd),rv=rightV(gd.dd);
  const gDoor={x:gp[0]+rv[0]*2.5,y:gp[1]+rv[1]*2.5};
  const req={id:ride.nextId++,rider,gps,sigma:8,t:simT,gpsStopErr:Math.hypot(gDoor.x-p.x,gDoor.y-p.y),where:destName(destFromPoint(p.x,p.y)),dest:opts.dest||null,user:!!opts.user};
  rider.reqId=req.id;
  if(req.user){ride.queue.unshift(req);
    if(ride.phase==='toPickup'&&ride.req&&!ride.req.user){const old=ride.req;old.rider.isRider=false;old.rider.mode='script';old.rider.path=[{x:old.rider.x,y:old.rider.y}];old.rider.pi=1;old.rider.scripted=true;old.rider.born=simT;old.rider.done=true;old.rider.col=pick(SHIRTS);
      ride.req=null;ride.phase='idle';logE(`↪ Auto request #${old.id} handed to another taxi — your booking takes priority`,'info');}
    logE(`📱 YOUR ride #${req.id} — pickup on ${req.where} → drop-off on ${destName(req.dest)}${ride.phase==='idle'?'':' (queued after current trip)'}`,'info');}
  else{ride.queue.push(req);logE(`📱 Ride request #${req.id} — rider on ${req.where}; phone GPS ±8 m`,'info');}
  return req;
}
function pfInit(c,sig){const n=500,px=new Float32Array(n),py=new Float32Array(n),w=new Float32Array(n).fill(1/n);
  for(let i=0;i<n;i++){const s=snapSidewalk(c.x+gauss()*sig,c.y+gauss()*sig);px[i]=s[0];py[i]=s[1];}return{n,px,py,w};}
function pfUpdate(pf,me){
  let sum=0;const ax=ego.x,ay=ego.y;
  for(let i=0;i<pf.n;i++){const dx=pf.px[i]-ax,dy=pf.py[i]-ay,d=Math.hypot(dx,dy);
    let l=Math.exp(-((me.rssi-rssiModel(d))**2)/(2*(RF.sh+1.5)**2));
    if(me.aoa!=null){const da=angDiff(Math.atan2(dy,dx),me.aoa);l*=Math.exp(-da*da/(2*(RF.aoa*1.6)**2))+.002;}
    pf.w[i]*=l+1e-9;sum+=pf.w[i];}
  if(!(sum>0)){pf.w.fill(1/pf.n);return;}
  let ne=0;for(let i=0;i<pf.n;i++){pf.w[i]/=sum;ne+=pf.w[i]*pf.w[i];}ne=1/ne;
  if(ne<pf.n/2){const nx=new Float32Array(pf.n),ny=new Float32Array(pf.n),st=1/pf.n;let r0=Math.random()*st,c=pf.w[0],j=0;
    for(let i=0;i<pf.n;i++){const u=r0+i*st;while(u>c&&j<pf.n-1){j++;c+=pf.w[j];}const s=snapSidewalk(pf.px[j]+gauss()*.35,pf.py[j]+gauss()*.35);nx[i]=s[0];ny[i]=s[1];}
    pf.px=nx;pf.py=ny;pf.w.fill(1/pf.n);}
}
function pfEst(pf){let mx=0,my=0;for(let i=0;i<pf.n;i++){mx+=pf.w[i]*pf.px[i];my+=pf.w[i]*pf.py[i];}
  let v=0;for(let i=0;i<pf.n;i++)v+=pf.w[i]*((pf.px[i]-mx)**2+(pf.py[i]-my)**2);return{x:mx,y:my,sd:Math.sqrt(v)};}
const sameDest=(a,b)=>a&&b&&a.a.i===b.a.i&&a.a.j===b.a.j&&a.dd===b.dd;
function nearIntersection(){return Math.abs(ego.x-Math.round(ego.x/P)*P)<9&&Math.abs(ego.y-Math.round(ego.y/P)*P)<9;}
function doorPt(){const c=Math.cos(ego.th),s=Math.sin(ego.th);return{x:ego.x-s*1.6,y:ego.y+c*1.6};}
function rideStep(dt){
  const R=ride,rem=route?(route.n-1-ego.pi)*DS:1e9;
  const pullT=((R.phase==='toPickup'||R.phase==='toDropoff')&&rem<24)||R.phase==='boarding'||R.phase==='alighting'?.6:0;
  ego.pull+=clamp(pullT-ego.pull,-.6*dt,.6*dt);
  if(R.phase==='idle'){
    if(!R.queue.length){R.idleT+=dt;if(R.auto&&R.idleT>2.5){R.idleT=0;makeRequest();}return;}
    const req=R.queue.shift();R.req=req;R.pf=null;R.est=null;R.meas=null;
    const dest=destFromPoint(req.gps.x,req.gps.y),r=planRoute(ego.x,ego.y,ego.d,dest);
    if(!r){logE(`Could not route to rider #${req.id} — request cancelled`,'warn');actors=actors.filter(o=>o!==req.rider);R.req=null;return;}
    R.dest=dest;R.phase='toPickup';setRoute(r,`🚕 Dispatched to rider #${req.id} (heading for the phone's GPS fix)`,'Pickup');return;}
  const req=R.req,rd=req.rider;
  if(R.phase==='toPickup'){
    const d=Math.hypot(rd.x-ego.x,rd.y-ego.y);
    if(d<RF.range&&simT-R.lastMeas>=.1){R.lastMeas=simT;
      const rssi=rssiAt(d)+gauss()*RF.sh,aoa=d<RF.uwb?Math.atan2(rd.y-ego.y,rd.x-ego.x)+gauss()*RF.aoa:null;
      R.meas={rssi,aoa,rng:Math.pow(10,(RF_MODEL.P0-rssi)/(10*RF_MODEL.n))};OUT.rf.push({t:+simT.toFixed(2),d:+d.toFixed(2),rssi:+rssi.toFixed(2)});
      if(!R.pf){R.pf=pfInit(req.gps,req.sigma);logE(`📡 RF lock on rider #${req.id}'s phone at ${d.toFixed(0)} m — BLE RSSI ${rssi.toFixed(0)} dBm`,'info');}
      pfUpdate(R.pf,R.meas);const e=pfEst(R.pf),sn=snapSidewalk(e.x,e.y);R.est={x:sn[0],y:sn[1],sd:e.sd};}
    if(R.est&&R.est.sd<6&&simT-R.lastReplan>.5&&!nearIntersection()&&rem>4){R.lastReplan=simT;
      const nd=destFromPoint(R.est.x,R.est.y);
      if(sameDest(nd,R.dest)){if(Math.abs(nd.frac-R.dest.frac)*P>1.2){const r=planRoute(ego.x,ego.y,ego.d,nd);if(r&&r.len<rem+25){R.dest=nd;replaceRoute(r);}}}
      else if(R.est.sd<3.5){const r=planRoute(ego.x,ego.y,ego.d,nd);if(r){R.dest=nd;replaceRoute(r);logE(`📡 RF correction — rider is really on ${destName(nd)}; re-routing (${r.len.toFixed(0)} m)`,'warn');}}}
    if(rem<1.5&&ego.v<.3){const dp=doorPt(),err=Math.hypot(dp.x-rd.x,dp.y-rd.y);R.stopErr.push(err);R.gpsErr.push(req.gpsStopErr);R.last={rf:err,gps:req.gpsStopErr};
      OUT.pickups.push({t:+simT.toFixed(1),id:req.id,user:!!req.user,x:+rd.x.toFixed(2),y:+rd.y.toFixed(2),gps_err:+Math.hypot(req.gps.x-rd.x,req.gps.y-rd.y).toFixed(2),door_rf:+err.toFixed(2),door_gps:+req.gpsStopErr.toFixed(2),wait_s:+(simT-req.t).toFixed(1),rf_model:RF_MODEL.src});logLeg();
      logE(`🛑 Pulled over for rider #${req.id} — door ${err.toFixed(1)} m from rider (GPS-only stop: ${req.gpsStopErr.toFixed(1)} m)`,'info');
      R.phase='boarding';rd.mode='walkTo';rd.goto=dp;rd.boarding=true;rd.sp=1.3;R.tB=simT;}
  }else if(R.phase==='boarding'){
    rd.goto=doorPt();
    if(rd.arrived||simT-R.tB>30){let r=null;
      if(req.dest)r=planRoute(ego.x,ego.y,ego.d,req.dest);
      if(!r)for(let t=0;t<40;t++){const c=planRoute(ego.x,ego.y,ego.d,randomDest());if(c&&c.len>150&&c.len<480){r=c;break;}}
      if(!r)r=planRoute(ego.x,ego.y,ego.d,randomDest());if(!r)return;
      actors=actors.filter(o=>o!==rd);R.pax=1;R.phase='toDropoff';setRoute(r,`👤 Rider #${req.id} on board`,'Drop-off');}
  }else if(R.phase==='toDropoff'){
    if(rem<1.5&&ego.v<.3){R.phase='alighting';R.pax=0;const dp=doorPt();rd.x=dp.x;rd.y=dp.y;rd.mode='walkTo';rd.boarding=true;rd.arrived=false;
      const sn=snapSidewalk(dp.x+(dp.x-ego.x)*1.5,dp.y+(dp.y-ego.y)*1.5);rd.goto={x:sn[0],y:sn[1]};actors.push(rd);R.tB=simT;stats.trips++;logLeg();
      logE(`✅ Trip ${stats.trips} complete — rider #${req.id} dropped off on ${route.name} (${odo.trip.toFixed(0)} m)`,'info');}
  }else if(R.phase==='alighting'){
    if(rd.arrived||simT-R.tB>10){rd.boarding=false;rd.mode='script';rd.path=[{x:rd.x,y:rd.y}];rd.pi=1;rd.scripted=true;rd.born=simT;rd.done=true;rd.col=pick(SHIRTS);R.phase='idle';R.req=null;R.idleT=0;}
  }
}

