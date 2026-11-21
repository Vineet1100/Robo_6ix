/* =============================================================================
 * 10-render.js — Drawing the map view
 * Camera, sprites for cars / streetcars / people / animals / the robotaxi, overlays (route, sensor rings, LiDAR, predicted paths, labels, street names), minimap.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== rendering ==============================
const cv=document.getElementById('cv'),ctx=cv.getContext('2d');
const mini=document.getElementById('mini'),mctx=mini.getContext('2d');
let cw=0,ch=0,dpr=1,Z=9,headingUp=true,showPred=true,showLidar=true,showRoute=true,showOdo=true,showDist=true;
const radar=document.getElementById('radar'),rctx=radar.getContext('2d');
const cam={x:0,y:0,ang:-Math.PI/2};
function resize(){const r=document.getElementById('tilt').getBoundingClientRect();dpr=Math.min(2,window.devicePixelRatio||1);cw=r.width;ch=r.height;cv.width=cw*dpr;cv.height=ch*dpr;}
window.addEventListener('resize',resize);
const camF=()=>headingUp?.64:.5;
function w2s(x,y){const dx=x-cam.x,dy=y-cam.y,c=Math.cos(cam.ang),s=Math.sin(cam.ang);return[cw/2+(dx*c-dy*s)*Z,ch*camF()+(dx*s+dy*c)*Z];}
function drawWalker(c,a){
  const t=simT*6+a.phase,moving=Math.hypot(a.vx||0,a.vy||0)>.05;
  c.save();c.translate(a.x,a.y);
  c.fillStyle='rgba(0,0,0,.33)';c.beginPath();c.ellipse(.22,.22,a.kind==='deer'?.85:a.kind==='dog'||a.kind==='raccoon'?.45:.32,a.kind==='deer'?.4:.25,a.hd,0,7);c.fill();
  c.rotate(a.hd);
  if(a.kind==='ped'||a.kind==='child'){const s=a.kind==='child'?.75:1;c.scale(s,s);
    if(moving){c.fillStyle='#2a2a2a';const sw=Math.sin(t)*.18;c.fillRect(sw-.06,-.14,.14,.09);c.fillRect(-sw-.06,.05,.14,.09);}
    c.fillStyle=a.col;c.beginPath();c.ellipse(0,0,.16,.27,0,0,7);c.fill();
    c.fillStyle='#2b211a';c.beginPath();c.arc(.02,0,.12,0,7);c.fill();}
  else if(a.kind==='dog'||a.kind==='cat'){const s=a.kind==='cat'?.6:1;c.scale(s,s);
    c.strokeStyle=a.fur;c.lineWidth=.08;c.beginPath();c.moveTo(-.4,0);c.lineTo(-.65,Math.sin(t*1.5)*.15);c.stroke();
    c.fillStyle=a.fur;c.beginPath();c.ellipse(0,0,.42,.16,0,0,7);c.fill();c.beginPath();c.arc(.45,0,.14,0,7);c.fill();
    c.fillStyle=shade(a.fur,-.35);c.beginPath();c.ellipse(.42,-.12,.06,.04,0,0,7);c.ellipse(.42,.12,.06,.04,0,0,7);c.fill();}
  else if(a.kind==='raccoon'){c.fillStyle='#6f6a64';c.beginPath();c.ellipse(0,0,.36,.17,0,0,7);c.fill();
    for(let k=0;k<4;k++){c.fillStyle=k%2?'#2b2826':'#9a948c';c.beginPath();c.arc(-.4-k*.09,Math.sin(t*1.2)*.04,.075,0,7);c.fill();}
    c.fillStyle='#8b857d';c.beginPath();c.arc(.38,0,.13,0,7);c.fill();c.fillStyle='#1f1c1a';c.fillRect(.36,-.11,.07,.22);c.fillStyle='#e8e2d6';c.beginPath();c.arc(.5,0,.04,0,7);c.fill();}
  else if(a.kind==='deer'){c.fillStyle='#9a6a3f';c.beginPath();c.ellipse(0,0,.75,.28,0,0,7);c.fill();
    c.beginPath();c.ellipse(.82,0,.24,.12,0,0,7);c.fill();c.fillStyle='#f3eadb';c.beginPath();c.arc(-.72,0,.1,0,7);c.fill();
    c.strokeStyle='#5a3d22';c.lineWidth=.05;c.beginPath();c.moveTo(.9,-.05);c.lineTo(1.05,-.25);c.moveTo(.9,.05);c.lineTo(1.05,.25);c.stroke();}
  else if(a.kind==='ball'){c.rotate(simT*8);c.fillStyle='#e23b3b';c.beginPath();c.arc(0,0,.15,0,7);c.fill();c.fillStyle='#fff';c.fillRect(-.15,-.03,.3,.06);}
  c.restore();
}
function drawEgo(c){
  const p=egoPos();
  c.save();c.translate(p.x+.45,p.y+.45);c.rotate(ego.hd);c.fillStyle='rgba(0,0,0,.42)';rr(c,-2.2,-1.02,4.4,2.05,.7);c.fill();c.restore();
  c.save();c.translate(p.x,p.y);c.rotate(ego.hd);
  rr(c,-2.2,-1.02,4.4,2.05,.7);c.fillStyle='#eef2f3';c.fill();c.strokeStyle='#38e1c6';c.lineWidth=.09;c.stroke();
  c.fillStyle='#16202a';rr(c,1.05,-.86,.72,1.72,.25);c.fill();rr(c,-1.77,-.86,.72,1.72,.25);c.fill();
  c.fillStyle='#dfe6e8';rr(c,-1.0,-.82,2.0,1.64,.3);c.fill();
  c.fillStyle='#38e1c6';c.fillRect(-1.0,-.07,2.0,.14);
  c.fillStyle='#1b2229';c.beginPath();c.arc(0,0,.38,0,7);c.fill();c.strokeStyle='#38e1c6';c.lineWidth=.06;c.stroke();
  const sa=performance.now()/90;c.fillStyle='#7ff5e1';c.beginPath();c.arc(Math.cos(sa)*.24,Math.sin(sa)*.24,.07,0,7);c.fill();
  c.fillStyle='#1b2229';for(const[sx,sy]of[[2.05,.85],[2.05,-.85],[-2.05,.85],[-2.05,-.85]]){c.beginPath();c.arc(sx,sy,.13,0,7);c.fill();}
  c.fillStyle='#fffbe0';c.fillRect(2.1,-.7,.08,.4);c.fillRect(2.1,.3,.08,.4);
  c.fillStyle=ego.acc<-1?'#ff2a3a':'#8c1b24';c.fillRect(-2.18,-.7,.08,.4);c.fillRect(-2.18,.3,.08,.4);
  c.restore();
}
function bracket(c,x,y,ang,L,Wd,col,lw){
  c.save();c.translate(x,y);c.rotate(ang);c.strokeStyle=col;c.lineWidth=lw;const hx=L/2,hy=Wd/2,k=Math.min(L,Wd)*.35;
  c.beginPath();
  for(const[sx,sy]of[[1,1],[1,-1],[-1,1],[-1,-1]]){c.moveTo(sx*hx-sx*k,sy*hy);c.lineTo(sx*hx,sy*hy);c.lineTo(sx*hx,sy*hy-sy*k);}
  c.stroke();c.globalAlpha=.08;c.fillStyle=col;c.fillRect(-hx,-hy,L,Wd);c.restore();
}
function render(rdt){
  const ep=egoPos();
  cam.x=ep.x;cam.y=ep.y;
  const tgtA=headingUp?-Math.PI/2-ego.hd:0;cam.ang+=angDiff(cam.ang,tgtA)*Math.min(1,rdt*3.5);
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#3a4630';ctx.fillRect(0,0,cw,ch);
  ctx.save();ctx.translate(cw/2,ch*camF());ctx.scale(Z,Z);ctx.rotate(cam.ang);ctx.translate(-cam.x,-cam.y);
  ctx.imageSmoothingEnabled=true;
  ctx.drawImage(TEX,-M,-M,W+2*M,W+2*M);
  drawTrafficWorld(ctx,ep);
  const lw=1.4/Z;
  // sensor field
  ctx.strokeStyle='rgba(56,225,198,.22)';ctx.lineWidth=lw;
  for(let r=10;r<=RANGE;r+=10){ctx.beginPath();ctx.arc(ep.x,ep.y,r,0,7);ctx.stroke();}
  if(showLidar){const sa=performance.now()/600;ctx.fillStyle='rgba(56,225,198,.07)';ctx.beginPath();ctx.moveTo(ep.x,ep.y);ctx.arc(ep.x,ep.y,RANGE,sa,sa+.5);ctx.closePath();ctx.fill();}
  // route + destination
  if(route&&showRoute){const r=route;ctx.lineCap='round';ctx.lineJoin='round';
    ctx.strokeStyle='#2f6bff';ctx.globalAlpha=.55;ctx.lineWidth=1.3;ctx.beginPath();
    for(let q=ego.pi;q<r.n;q+=2){q===ego.pi?ctx.moveTo(r.px[q],r.py[q]):ctx.lineTo(r.px[q],r.py[q]);}ctx.lineTo(r.px[r.n-1],r.py[r.n-1]);ctx.stroke();
    ctx.globalAlpha=.9;ctx.strokeStyle='#9cc0ff';ctx.lineWidth=lw;ctx.stroke();ctx.globalAlpha=1;
    for(const nd of r.nodes){if(!nd.turn||nd.q<ego.pi)continue;ctx.fillStyle='#9cc0ff';ctx.beginPath();ctx.arc(r.px[nd.q],r.py[nd.q],.45,0,7);ctx.fill();}
    const dx=r.px[r.n-1],dy=r.py[r.n-1],pul=1+.25*Math.sin(performance.now()/250);
    ctx.fillStyle='rgba(47,107,255,.18)';ctx.beginPath();ctx.arc(dx,dy,3.2*pul,0,7);ctx.fill();
    ctx.strokeStyle='#6fa8ff';ctx.lineWidth=lw*1.5;ctx.beginPath();ctx.arc(dx,dy,2.2,0,7);ctx.stroke();
    ctx.fillStyle='#2f6bff';ctx.beginPath();ctx.arc(dx,dy,.9,0,7);ctx.fill();ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(dx,dy,.35,0,7);ctx.fill();}
  // risk hotspots found by DBSCAN in Python (scikit-learn)
  if(PY.showHot&&PY.analytics&&PY.analytics.hotspots){for(const h of PY.analytics.hotspots){ctx.fillStyle='rgba(255,77,94,.12)';ctx.strokeStyle='rgba(255,77,94,.7)';ctx.lineWidth=lw*1.5;ctx.setLineDash([1,.7]);
    ctx.beginPath();ctx.arc(h.x,h.y,h.radius,0,7);ctx.fill();ctx.stroke();ctx.setLineDash([]);}}
  // odometry trails
  if(showOdo){const T=odo.trail;const line=(arr,col,a,w)=>{if(arr.length<4)return;ctx.strokeStyle=col;ctx.globalAlpha=a;ctx.lineWidth=w;ctx.beginPath();ctx.moveTo(arr[0],arr[1]);for(let k=2;k<arr.length;k+=2)ctx.lineTo(arr[k],arr[k+1]);ctx.stroke();};
    line(T.tru,'#ffffff',.55,lw*1.6);line(T.raw,'#ff5bd1',.9,lw*1.6);line(T.fus,'#38e1c6',.9,lw*1.3);ctx.globalAlpha=1;
    ctx.save();ctx.translate(odo.raw.x,odo.raw.y);ctx.rotate(odo.raw.th);ctx.strokeStyle='#ff5bd1';ctx.lineWidth=lw*1.3;ctx.setLineDash([.5,.4]);ctx.strokeRect(-2.2,-1.02,4.4,2.05);ctx.setLineDash([]);ctx.restore();}
  // ride-hailing: riders, GPS fix, particle filter
  {const pul=(performance.now()/900)%1;
   for(const a of actors){if(!a.isRider||a.boarding)continue;ctx.strokeStyle='#b388ff';ctx.lineWidth=lw*1.4;
     for(let k=0;k<3;k++){const ph=(pul+k/3)%1;ctx.globalAlpha=(1-ph)*.8;ctx.beginPath();ctx.arc(a.x,a.y,.6+ph*4.5,0,7);ctx.stroke();}ctx.globalAlpha=1;}
   const R=ride;if(R.req&&R.phase==='toPickup'){const q=R.req;
     ctx.strokeStyle='rgba(255,255,255,.7)';ctx.setLineDash([.7,.5]);ctx.lineWidth=lw*1.3;ctx.beginPath();ctx.arc(q.gps.x,q.gps.y,q.sigma,0,7);ctx.stroke();ctx.setLineDash([]);
     ctx.fillStyle='rgba(255,255,255,.08)';ctx.fill();
     ctx.fillStyle='#ffffff';ctx.beginPath();ctx.moveTo(q.gps.x,q.gps.y);ctx.arc(q.gps.x,q.gps.y-1.2,.7,Math.PI*.8,Math.PI*2.2);ctx.closePath();ctx.fill();
     if(R.pf){const pf=R.pf;ctx.fillStyle='#c9a6ff';for(let i=0;i<pf.n;i+=2){ctx.globalAlpha=Math.min(.9,.15+pf.w[i]*pf.n*.5);ctx.fillRect(pf.px[i]-.09,pf.py[i]-.09,.18,.18);}ctx.globalAlpha=1;}
     if(R.est){const e=R.est;ctx.strokeStyle='#b388ff';ctx.lineWidth=lw*1.6;ctx.beginPath();ctx.arc(e.x,e.y,Math.max(.4,2*e.sd),0,7);ctx.stroke();
       ctx.fillStyle='#b388ff';ctx.beginPath();ctx.moveTo(e.x,e.y-.7);ctx.lineTo(e.x+.7,e.y);ctx.lineTo(e.x,e.y+.7);ctx.lineTo(e.x-.7,e.y);ctx.closePath();ctx.fill();}
     if(R.meas){const rd=q.rider;ctx.strokeStyle='rgba(179,136,255,.8)';ctx.lineWidth=lw;ctx.setLineDash([.3,.5]);ctx.beginPath();ctx.moveTo(ego.x,ego.y);ctx.lineTo(rd.x,rd.y);ctx.stroke();ctx.setLineDash([]);}}}
  // planned trajectory
  if(decision.traj){const tr=decision.traj,col=riskCol(decision.results.find(r=>r.act===decision.act)?.Rmax||0);
    ctx.lineCap='round';ctx.lineJoin='round';ctx.strokeStyle=col;ctx.globalAlpha=.22;ctx.lineWidth=2.1;ctx.beginPath();
    for(let i=0;i<=NS;i++){const x=tr.pts[2*i],y=tr.pts[2*i+1];i?ctx.lineTo(x,y):ctx.moveTo(x,y);}ctx.stroke();
    ctx.globalAlpha=.9;ctx.lineWidth=lw*1.3;ctx.setLineDash([.8,.6]);ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=1;
    for(let i=10;i<=NS;i+=10){ctx.fillStyle=col;ctx.beginPath();ctx.arc(tr.pts[2*i],tr.pts[2*i+1],.18,0,7);ctx.fill();}}
  // agents
  const vis=actors.filter(o=>Math.abs(o.x-ep.x)<110&&Math.abs(o.y-ep.y)<110);
  for(const a of vis){if(a.kind!=='car')continue;
    ctx.save();ctx.translate(a.x+.45,a.y+.45);ctx.rotate(a.hd);ctx.fillStyle='rgba(0,0,0,.38)';rr(ctx,-a.L/2,-a.Wd/2,a.L,a.Wd,.45);ctx.fill();ctx.restore();
    ctx.save();ctx.translate(a.x,a.y);ctx.rotate(a.hd);a.tram?drawTram(ctx,a):drawCarTop(ctx,a.L,a.Wd,a.color,a.acc<-1);ctx.restore();}
  for(const a of vis)if(a.kind!=='car'&&a.kind!=='ego')drawWalker(ctx,a);
  drawEgo(ctx);
  // pure pursuit geometry
  if(showRoute&&ego.pp){const pp=ego.pp;
    ctx.strokeStyle='rgba(255,255,255,.35)';ctx.lineWidth=lw;ctx.setLineDash([.4,.4]);ctx.beginPath();ctx.arc(pp.rx,pp.ry,pp.Ld,0,7);ctx.stroke();ctx.setLineDash([]);
    ctx.strokeStyle='#ffe066';ctx.lineWidth=lw*1.2;ctx.beginPath();ctx.moveTo(pp.rx,pp.ry);ctx.lineTo(pp.lx,pp.ly);ctx.stroke();
    // commanded arc
    let ax=pp.rx,ay=pp.ry,th=ego.th;ctx.strokeStyle='#ffb020';ctx.lineWidth=lw*2;ctx.beginPath();ctx.moveTo(ax,ay);
    for(let k=0;k<24;k++){const st=pp.Ld/24;ax+=Math.cos(th)*st;ay+=Math.sin(th)*st;th+=pp.kappa*st;ctx.lineTo(ax,ay);}ctx.stroke();
    ctx.fillStyle='#ffe066';ctx.beginPath();ctx.arc(pp.lx,pp.ly,.4,0,7);ctx.fill();
    ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(pp.rx,pp.ry,.18,0,7);ctx.fill();}
  // perception overlays
  const tracks=decision.tracks;
  if(showPred)for(const tk of tracks){const o=tk.o,m=tk.m;if(Math.hypot(o.vx||0,o.vy||0)<.05&&!(o.kind==='car'&&o.v>.1))continue;
    const col=riskCol(tk.pPlan);ctx.strokeStyle=col;ctx.globalAlpha=.75;ctx.lineWidth=lw;ctx.setLineDash([.5,.5]);ctx.beginPath();
    for(let i=0;i<=30;i+=2){const x=m.pts[2*i],y=m.pts[2*i+1];i?ctx.lineTo(x,y):ctx.moveTo(x,y);}ctx.stroke();ctx.setLineDash([]);
    if(o.kind!=='car'){const sg=m.s0+m.k*2;ctx.globalAlpha=.12;ctx.fillStyle=col;ctx.beginPath();ctx.arc(m.pts[40],m.pts[41],sg+m.r,0,7);ctx.fill();}
    ctx.globalAlpha=1;}
  for(const tk of tracks){const o=tk.o,col=CCOL[o.kind];
    if(o.kind==='car')bracket(ctx,o.x,o.y,o.hd,o.L+.5,o.Wd+.5,col,lw*1.4);
    else{const b=SIZE[o.kind].b;bracket(ctx,o.x,o.y,o.hd,b*(o.kind==='deer'||o.kind==='dog'?1.1:1),o.kind==='deer'?b*.55:o.kind==='dog'?b*.5:b,col,lw*1.4);}
    if(showLidar){const n=o.kind==='car'?18:7,ext=o.kind==='car'?o.L/2:SIZE[o.kind].r;
      for(let k=0;k<n;k++){const ang=Math.random()*6.283,rad=ext*(o.kind==='car'?.9:1);const px=o.x+Math.cos(ang)*rad*(o.kind==='car'?1:1),py=o.y+Math.sin(ang)*rad*(o.kind==='car'?.45:1);
        ctx.fillStyle=`hsl(${170-tk.dist*2},90%,62%)`;ctx.fillRect(px-.06,py-.06,.12,.12);}}
  }
  if(decision.crit){const o=decision.crit.o;ctx.strokeStyle='#ff4d5e';ctx.lineWidth=lw*1.6;ctx.setLineDash([.6,.4]);ctx.beginPath();ctx.moveTo(ep.x,ep.y);ctx.lineTo(o.x,o.y);ctx.stroke();ctx.setLineDash([]);
    const pul=1+.3*Math.sin(performance.now()/120);ctx.beginPath();ctx.arc(o.x,o.y,(SIZE[o.kind].r+1)*pul,0,7);ctx.stroke();}
  ctx.restore();
  // labels (screen space)
  ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.textBaseline='middle';
  drawSigns(ctx,ep);
  const lab=tracks.filter((t,i)=>i<14||t.pPlan>.1);
  for(const tk of lab){const o=tk.o;const[sx,sy]=w2s(o.x,o.y);if(sx<-50||sy<-50||sx>cw+50||sy>ch+50)continue;
    const l1=`${o.tram?'TTC':CLS[o.kind]} ${o.id}  ${(o.conf*100).toFixed(0)}%`,l2=`${tk.dist.toFixed(1)}m  P ${Math.round(tk.pPlan*100)}%`;
    ctx.font='600 10.5px "JetBrains Mono",monospace';const w=Math.max(ctx.measureText(l1).width,ctx.measureText(l2).width)+12;
    const bx=sx-w/2,by=sy-(o.kind==='car'?34:28)-14;
    ctx.fillStyle='rgba(8,12,16,.82)';rr(ctx,bx,by,w,28,5);ctx.fill();
    ctx.fillStyle=CCOL[o.kind];ctx.fillRect(bx,by+4,2.5,20);
    ctx.fillStyle='#e6eef5';ctx.fillText(l1,bx+7,by+8.5);
    ctx.fillStyle=riskCol(tk.pPlan);ctx.fillText(l2,bx+7,by+20);
    ctx.strokeStyle='rgba(255,255,255,.25)';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(sx,by+28);ctx.lineTo(sx,sy-8);ctx.stroke();}
  {const[sx,sy]=w2s(ep.x,ep.y);ctx.font='700 10px "JetBrains Mono",monospace';const t='EGO';const w=ctx.measureText(t).width+10;
    ctx.fillStyle='#38e1c6';rr(ctx,sx-w/2,sy+26,w,16,4);ctx.fill();ctx.fillStyle='#062521';ctx.fillText(t,sx-w/2+5,sy+34);}
  // distance lines to nearest of each class + destination
  if(showDist){const groups={veh:null,ped:null,ani:null};
    for(const tk of tracks){const g=tk.o.kind==='car'?'veh':(tk.o.kind==='ped'||tk.o.kind==='child')?'ped':tk.o.kind==='ball'?null:'ani';if(g&&(!groups[g]||tk.dist<groups[g].dist))groups[g]=tk;}
    const[ex,ey]=w2s(ep.x,ep.y);
    for(const g of ['veh','ped','ani']){const tk=groups[g];if(!tk)continue;const[ox,oy]=w2s(tk.o.x,tk.o.y);const col=g==='veh'?'#4da3ff':g==='ped'?'#ffd84d':'#ff8a3d';
      ctx.strokeStyle=col;ctx.globalAlpha=.8;ctx.lineWidth=1.2;ctx.setLineDash([4,3]);ctx.beginPath();ctx.moveTo(ex,ey);ctx.lineTo(ox,oy);ctx.stroke();ctx.setLineDash([]);ctx.globalAlpha=1;
      const mx=(ex+ox)/2,my=(ey+oy)/2,t=`${tk.dist.toFixed(1)} m`;ctx.font='700 10.5px "JetBrains Mono",monospace';const w=ctx.measureText(t).width+8;
      ctx.fillStyle=col;rr(ctx,mx-w/2,my-8,w,16,4);ctx.fill();ctx.fillStyle='#0a0e13';ctx.fillText(t,mx-w/2+4,my);}}
  if(route&&showRoute){const r=route,[sx,sy]=w2s(r.px[r.n-1],r.py[r.n-1]);const rem=Math.max(0,(r.n-1-ego.pi)*DS);
    const cx=clamp(sx,70,cw-70),cy=clamp(sy,52,ch-20),t=(ride.phase==='boarding'||ride.phase==='alighting')?`${(r.kind||'').toUpperCase()} · stopped`:`${(r.kind||'DEST').toUpperCase()} · ${rem.toFixed(0)} m`;
    ctx.font='700 11px "JetBrains Mono",monospace';const w=ctx.measureText(t).width+14;
    ctx.fillStyle='#2f6bff';rr(ctx,cx-w/2,cy-34,w,20,5);ctx.fill();ctx.fillStyle='#fff';ctx.fillText(t,cx-w/2+7,cy-24);}
  // street names
  {ctx.font='600 11px Inter';ctx.textAlign='center';
   for(let r=0;r<=N;r++)for(let k=-1;k<=N;k++){for(const hz of [true,false]){const along=k*P+P/2,wx=hz?along:r*P,wy=hz?r*P:along;
     if(Math.abs(wx-ep.x)>90||Math.abs(wy-ep.y)>90)continue;const[sx,sy]=w2s(wx,wy);if(sx<-80||sy<-30||sx>cw+80||sy>ch+30)continue;
     let ang=cam.ang+(hz?0:Math.PI/2);while(ang>Math.PI/2)ang-=Math.PI;while(ang<-Math.PI/2)ang+=Math.PI;
     ctx.save();ctx.translate(sx,sy);ctx.rotate(ang);const t=hz?HNAME[r]:VNAME[r];ctx.lineWidth=3;ctx.strokeStyle='rgba(10,14,18,.75)';ctx.strokeText(t,0,4);ctx.fillStyle='rgba(255,255,255,.88)';ctx.fillText(t,0,4);ctx.restore();}}
   ctx.textAlign='left';}
  // ride labels
  {const R=ride;for(const a of actors){if(!a.isRider||a.boarding)continue;const[sx,sy]=w2s(a.x,a.y);if(sx<-40||sy<-40||sx>cw+40||sy>ch+40)continue;
     const t=a.user?'📱 YOU':`📱 RIDER #${a.reqId}`;ctx.font='700 10.5px "JetBrains Mono",monospace';const w=ctx.measureText(t).width+10;ctx.fillStyle=a.user?'#38e1c6':'#b388ff';rr(ctx,sx-w/2,sy+14,w,17,4);ctx.fill();ctx.fillStyle='#14082b';ctx.fillText(t,sx-w/2+5,sy+22.5);}
   if(R.req&&R.phase==='toPickup'&&R.meas){const rd=R.req.rider,[ax,ay]=w2s(ego.x,ego.y),[bx,by]=w2s(rd.x,rd.y),mx=(ax+bx)/2,my=(ay+by)/2;
     const t=`${R.meas.rssi.toFixed(0)} dBm${R.meas.aoa!=null?' · UWB':''}`;ctx.font='700 10px "JetBrains Mono",monospace';const w=ctx.measureText(t).width+10;
     ctx.fillStyle='rgba(20,8,43,.9)';rr(ctx,mx-w/2,my-9,w,18,4);ctx.fill();ctx.strokeStyle='#b388ff';ctx.lineWidth=1;ctx.stroke();ctx.fillStyle='#d9c4ff';ctx.fillText(t,mx-w/2+5,my);}
   if(R.req&&R.phase==='toPickup'){const g=R.req.gps,[gx,gy]=w2s(g.x,g.y);ctx.font='600 10px "JetBrains Mono",monospace';ctx.fillStyle='rgba(255,255,255,.85)';ctx.fillText('phone GPS ±8 m',gx+8,gy-14);}}
  // minimap
  mctx.drawImage(miniBase,0,0);const ms=340/(W+2*M);drawTrafficMini(mctx,ms);
  for(const a of actors){const x=(a.x+M)*ms,y=(a.y+M)*ms;
    if(a.isRider&&!a.boarding){mctx.fillStyle='#b388ff';mctx.beginPath();mctx.arc(x,y,6,0,7);mctx.fill();continue;}
    mctx.fillStyle=a===ego?'#38e1c6':a.kind==='car'?'#ffffff':a.kind==='ped'||a.kind==='child'?'#ffd84d':'#ff8a3d';
    const s=a===ego?7:a.kind==='car'?4:3;mctx.fillRect(x-s/2,y-s/2,s,s);}
  mctx.strokeStyle='rgba(56,225,198,.9)';mctx.lineWidth=2;mctx.beginPath();mctx.arc((ep.x+M)*ms,(ep.y+M)*ms,RANGE*ms,0,7);mctx.stroke();
  if(PY.showHot&&PY.analytics&&PY.analytics.hotspots)for(const h of PY.analytics.hotspots){mctx.strokeStyle='#ff4d5e';mctx.lineWidth=2;mctx.beginPath();mctx.arc((h.x+M)*ms,(h.y+M)*ms,Math.max(5,h.radius*ms),0,7);mctx.stroke();}
  if(route){const r=route;mctx.strokeStyle='#4f8dff';mctx.lineWidth=3;mctx.beginPath();for(let q=ego.pi;q<r.n;q+=4){const x=(r.px[q]+M)*ms,y=(r.py[q]+M)*ms;q===ego.pi?mctx.moveTo(x,y):mctx.lineTo(x,y);}mctx.stroke();
    const x=(r.px[r.n-1]+M)*ms,y=(r.py[r.n-1]+M)*ms;mctx.fillStyle='#2f6bff';mctx.beginPath();mctx.arc(x,y,7,0,7);mctx.fill();mctx.fillStyle='#fff';mctx.beginPath();mctx.arc(x,y,3,0,7);mctx.fill();}
  drawRadar();
}

function drawRadar(){
  const S=340,c=S/2,R=150,sc=R/RANGE;rctx.setTransform(1,0,0,1,0,0);rctx.clearRect(0,0,S,S);
  rctx.fillStyle='#0b1218';rctx.beginPath();rctx.arc(c,c,R+8,0,7);rctx.fill();
  rctx.strokeStyle='rgba(56,225,198,.18)';rctx.lineWidth=1.5;rctx.font='600 15px "JetBrains Mono",monospace';rctx.fillStyle='rgba(124,144,163,.8)';
  for(let r=15;r<=RANGE;r+=15){rctx.beginPath();rctx.arc(c,c,r*sc,0,7);rctx.stroke();rctx.fillText(r+'m',c+4,c-r*sc+14);}
  rctx.beginPath();rctx.moveTo(c,c-R);rctx.lineTo(c,c+R);rctx.moveTo(c-R,c);rctx.lineTo(c+R,c);rctx.stroke();
  const th=ego.th,co=Math.cos(th),si=Math.sin(th);
  if(route){const r=route,dx=r.px[r.n-1]-ego.x,dy=r.py[r.n-1]-ego.y,f=dx*co+dy*si,l=-dx*si+dy*co,ang=Math.atan2(l,f),dist=Math.hypot(dx,dy);
    const rr2=Math.min(dist*sc,R),x=c+Math.sin(ang)*rr2,y=c-Math.cos(ang)*rr2;
    rctx.strokeStyle='#4f8dff';rctx.lineWidth=3;rctx.setLineDash([6,5]);rctx.beginPath();rctx.moveTo(c,c);rctx.lineTo(x,y);rctx.stroke();rctx.setLineDash([]);
    rctx.fillStyle='#2f6bff';rctx.beginPath();rctx.arc(x,y,9,0,7);rctx.fill();rctx.fillStyle='#fff';rctx.beginPath();rctx.arc(x,y,3.5,0,7);rctx.fill();}
  for(const tk of decision.tracks){const x=c+tk.lat*sc,y=c-tk.fwd*sc;
    if(tk.pNo>.1){rctx.strokeStyle=riskCol(tk.pNo);rctx.lineWidth=2.5;rctx.beginPath();rctx.arc(x,y,10,0,7);rctx.stroke();}
    rctx.fillStyle=CCOL[tk.o.kind];if(tk.o.kind==='car'){rctx.save();rctx.translate(x,y);const oh=DIR[tk.o.d];rctx.rotate(Math.atan2(-oh[0]*si+oh[1]*co,oh[0]*co+oh[1]*si)-Math.PI/2+Math.PI);rctx.fillRect(-4,-8,8,16);rctx.restore();}
    else{rctx.beginPath();rctx.arc(x,y,4.5,0,7);rctx.fill();}}
  rctx.fillStyle='#38e1c6';rctx.beginPath();rctx.moveTo(c,c-12);rctx.lineTo(c+8,c+9);rctx.lineTo(c-8,c+9);rctx.closePath();rctx.fill();
  rctx.fillStyle='#7c90a3';rctx.font='700 14px Inter';rctx.fillText('FWD',c-15,18);
}
function fmtRel(f,l){return `${f>=0?'↑':'↓'}${Math.abs(f).toFixed(1)} ${l>=0?'R':'L'}${Math.abs(l).toFixed(1)}`;}
function navText(){
  const r=route;if(!r)return'No route';
  if(ride.phase==='boarding')return'<span class="ar">◉</span>Pulled over — rider boarding';
  if(ride.phase==='alighting')return'<span class="ar">◉</span>Arrived — rider getting out';
  if(ride.phase==='idle')return'<span class="ar">…</span>Waiting for next ride request';
  const nx=r.nodes.find(n=>n.turn&&n.q>ego.pi-4);const rem=(r.n-1-ego.pi)*DS;
  if(nx){const d=Math.max(0,(nx.q-ego.pi)*DS);return `<span class="ar">${nx.turn==='right'?'↱':'↰'}</span>${d<3?'Turning':'Turn'} ${nx.turn} onto ${nx.onto}${d<3?'':' in '+d.toFixed(0)+' m'}`;}
  return `<span class="ar">⬆</span>Continue — ${(r.kind||'destination').toLowerCase()} in ${rem.toFixed(0)} m`;
}
function locText(){
  const d=ego.d,hx=horiz(d),al=hx?ego.x:ego.y,perp=hx?ego.y:ego.x,ri=clamp(Math.round(perp/P),0,N);
  const road=hx?HNAME[ri]:VNAME[ri],nd=nodeAhead(ego.x,ego.y,d);const ni=Math.round(al/P);
  if(Math.abs(al-ni*P)<6&&Math.abs(perp-ri*P)<6)return `In the intersection of <b>${hx?HNAME[ri]:VNAME[ri]}</b> & <b>${hx?VNAME[clamp(ni,0,N)]:HNAME[clamp(ni,0,N)]}</b>`;
  const cross=inGrid(nd)?(hx?VNAME[nd.i]:HNAME[nd.j]):null,dist=Math.abs((hx?nd.i:nd.j)*P-al);
  return `On <b>${road}</b>, ${DNAME[d]}, lane offset ${(ego.cte>=0?'+':'')}${ego.cte.toFixed(2)} m${cross?` · <b>${dist.toFixed(0)} m</b> before ${cross}`:''}`;
}
const PHASES=['Requested','En route','RF lock','Pulled over','Boarding','To destination','Dropped off'];
const fm=v=>v==null?'—':v.toFixed(1)+' m';
function updateRideUI(){
  const R=ride;let idx=-1;
  if(R.phase==='idle')idx=R.queue.length?0:-1;else if(R.phase==='toPickup')idx=R.pf?2:1;else if(R.phase==='boarding')idx=4;else if(R.phase==='toDropoff')idx=5;else if(R.phase==='alighting')idx=6;
  $('rSteps').innerHTML=PHASES.map((p,i)=>`<span class="${i===idx?'on':i<idx?'done':''}">${p}</span>`).join('');
  const q=R.req;
  $('rWho').textContent=q?`${q.user?'Your ride':'Rider'} #${q.id} · ${q.where}`:'Waiting for a request…';
  $('rPh').textContent=R.pax?'1 passenger':'empty';
  if(q&&R.phase==='toPickup'){const rd=q.rider;$('rGps').textContent=fm(Math.hypot(q.gps.x-rd.x,q.gps.y-rd.y));
    $('rEst').textContent=R.est?fm(Math.hypot(R.est.x-rd.x,R.est.y-rd.y)):'no lock';$('rSd').textContent=R.est?fm(R.est.sd):'—';
    $('rRssi').textContent=R.meas?R.meas.rssi.toFixed(0)+' dBm':'out of range';$('rRng').textContent=R.meas?fm(R.meas.rng):'—';
    $('rAoa').textContent=R.meas&&R.meas.aoa!=null?(angDiff(ego.th,R.meas.aoa)*180/Math.PI).toFixed(0)+'°':'—';}
  else{['rGps','rEst','rSd','rRssi','rRng','rAoa'].forEach(id=>$(id).textContent='—');}
  const avg=a=>a.length?a.reduce((x,y)=>x+y,0)/a.length:null;
  $('rLast').textContent=R.last?fm(R.last.rf):'—';$('rAvg').textContent=fm(avg(R.stopErr));$('rAvgG').textContent=fm(avg(R.gpsErr));
  $('rQueue').textContent=R.queue.length?'Queue: '+R.queue.map(x=>`#${x.id} (${x.where})`).join(', '):'Queue empty.';
}
function chartBase(c,lo,hi,tick){const W2=c.canvas.width,H2=c.canvas.height;c.clearRect(0,0,W2,H2);c.strokeStyle='rgba(124,144,163,.15)';c.lineWidth=1;c.fillStyle='rgba(124,144,163,.8)';c.font='600 18px "JetBrains Mono",monospace';
  for(let v=Math.ceil(lo/tick)*tick;v<=hi;v+=tick){const y=H2-(v-lo)/(hi-lo)*H2;c.beginPath();c.moveTo(0,y);c.lineTo(W2,y);c.stroke();c.fillText(v.toFixed(tick<1?1:0),4,y-3);}
  return(v)=>H2-(v-lo)/(hi-lo)*H2;}
function plotLine(c,xs,ys,Y,col,w,dash){c.strokeStyle=col;c.lineWidth=w;c.setLineDash(dash||[]);c.beginPath();xs.forEach((x,i)=>{const y=Y(ys[i]);i?c.lineTo(x,y):c.moveTo(x,y);});c.stroke();c.setLineDash([]);}
function updatePidUI(){
  const H=ctl.hist;if(H.length>2){const t1=H[H.length-1][0],t0=t1-20;
    const c1=$('chSpd').getContext('2d'),Wd=712,xs=H.map(h=>(h[0]-t0)/20*Wd);
    const Y=chartBase(c1,-6,14,2);
    H.forEach((h,i)=>{const u=h[3];c1.fillStyle=u>=0?'rgba(61,220,132,.45)':'rgba(255,77,94,.5)';const y0=Y(0),y1=Y(u*6);c1.fillRect(xs[i]-1.5,Math.min(y0,y1),3,Math.abs(y1-y0));});
    plotLine(c1,xs,H.map(h=>h[1]),Y,'#ffffff',2,[8,6]);plotLine(c1,xs,H.map(h=>h[2]),Y,'#38e1c6',3);
    c1.fillStyle='rgba(124,144,163,.8)';c1.fillText('m/s · bars = u×6',Wd-190,20);
    const c2=$('chPid').getContext('2d'),Y2=chartBase(c2,-1.5,1.5,.5);
    plotLine(c2,xs,H.map(h=>clamp(h[7],-1.5,1.5)),Y2,'#7c90a3',2,[6,5]);plotLine(c2,xs,H.map(h=>clamp(h[4],-1.5,1.5)),Y2,'#ffd84d',2.5);
    plotLine(c2,xs,H.map(h=>h[5]),Y2,'#b388ff',2.5);plotLine(c2,xs,H.map(h=>clamp(h[6],-1.5,1.5)),Y2,'#4dd0ff',2);plotLine(c2,xs,H.map(h=>h[3]),Y2,'#ffffff',2);}
  const u=ctl.u;
  $('cRef').textContent=(ctl.vref*3.6).toFixed(1)+' km/h';$('cErr').textContent=(ctl.e>=0?'+':'')+ctl.e.toFixed(2)+' m/s';
  $('cU').textContent=u>=0?'thr '+(u*100).toFixed(0)+'%':'brk '+(-u*100).toFixed(0)+'%';$('cU').style.color=u>=0?'#3ddc84':'#ff4d5e';
  $('cP').textContent=ctl.P.toFixed(3);$('cI').textContent=ctl.I.toFixed(3);$('cD').textContent=ctl.D.toFixed(3);$('cFF').textContent=ctl.ff.toFixed(3);
  $('cAcc').textContent=ego.acc.toFixed(2)+' m/s²';$('cGr').textContent=(ctl.grade*100).toFixed(1)+'%';
  $('pWind').style.display=ctl.clamp?'':'none';
  const z=ctl.zn;if(z){
    $('zCoord').innerHTML=`At coordinate <b class="mono">(${ego.x.toFixed(1)}, ${ego.y.toFixed(1)}) m</b> · ${z.mode==='traction'?'accelerating / cruising':'braking'} · v ${ego.v.toFixed(1)} m/s · mass ${mass()} kg`;
    $('zK').textContent=z.K.toFixed(0);$('zTau').textContent=z.tp.toFixed(0)+' s';$('zTh').textContent=(z.th*1000).toFixed(0)+' ms';
    $('zKu').textContent=z.Ku.toFixed(2);$('zTu').textContent=z.Tu.toFixed(2)+' s';$('zMu').textContent=z.mu.toFixed(2)+(z.mu<.6?' wet':' dry');$('zMu').style.color=z.mu<.6?'#6fa8ff':'';
    $('zRules').innerHTML=Object.keys(RULES).map(k=>{const g=ruleGains(z,k),on=k===ctl.rule;return `<tr data-r="${k}" style="cursor:pointer;${on?'background:#12302b':''}"><td style="color:${on?'#38e1c6':''}">${on?'▸ ':''}${RULES[k].name}</td><td>${g.Kp.toFixed(2)}</td><td>${g.Ki.toFixed(2)}</td><td>${g.Kd.toFixed(3)}</td><td>${g.Ti.toFixed(2)}/${g.Td.toFixed(2)}</td></tr>`;}).join('');}
  if(route&&route.zn){const Z=route.zn,c3=$('chZn').getContext('2d'),Wd=712,L=Math.max(route.len,1),mx=Math.max(...Z.kt,...Z.kb)*1.15;
    const Y=chartBase(c3,0,mx,mx>4?1:.5);const xs=Z.s.map(v=>v/L*Wd);
    Z.mu.forEach((m,i)=>{if(m<.6){c3.fillStyle='rgba(80,140,255,.22)';c3.fillRect(xs[i]-3,0,Wd/Z.s.length+4,208);}});
    plotLine(c3,xs,Z.kt,Y,'#4f8dff',3);plotLine(c3,xs,Z.kb,Y,'#ff8a3d',3);
    const ex=ego.pi*DS/L*Wd;c3.strokeStyle='#38e1c6';c3.lineWidth=3;c3.beginPath();c3.moveTo(ex,0);c3.lineTo(ex,208);c3.stroke();
    c3.fillStyle='rgba(124,144,163,.8)';c3.fillText(`Ku along ${route.kind.toLowerCase()} route (${L.toFixed(0)} m)`,Wd-360,20);}
  $('zLog').innerHTML=znLog.slice(-7).reverse().map(r=>`<tr><td>${r.x.toFixed(1)}, ${r.y.toFixed(1)}</td><td>${r.v.toFixed(1)}</td><td style="color:${r.mu<.6?'#6fa8ff':''}">${r.mu.toFixed(2)}</td><td>${r.Ku.toFixed(2)}</td><td>${r.Tu.toFixed(2)}</td><td>${r.Kp.toFixed(2)}</td><td>${r.Ki.toFixed(2)}</td><td>${r.Kd.toFixed(3)}</td></tr>`).join('')||'<tr><td colspan="8" style="text-align:left;color:#7c90a3">Logging starts after 5 m…</td></tr>';
}
