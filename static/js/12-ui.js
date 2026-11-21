/* =============================================================================
 * 12-ui.js — Side panels, buttons, keyboard, start-up
 * Updates every panel (decision engine, ride, PID charts, ZN tables, odometry, radar), wires up buttons and keys, and starts the app. Must load last.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== UI ==============================
const $=id=>document.getElementById(id);
const logBuf=[];
function logE(msg,cls){const t=simT;OUT.events.push({t:+simT.toFixed(1),message:msg.replace(/<[^>]+>/g,''),level:cls||''});const mm=String(Math.floor(t/60)).padStart(2,'0'),ss=(t%60).toFixed(1).padStart(4,'0');
  logBuf.unshift(`<div class="${cls||''}"><span class="t">${mm}:${ss}</span>${msg}</div>`);if(logBuf.length>60)logBuf.pop();$('log').innerHTML=logBuf.join('');}
function actClass(k){return k==='EMERGENCY'?'bad':(k==='BRAKE'||k.startsWith('NUDGE'))?'warn':k==='SLOW'?'warn':'';}
function updateUI(){
  const d=decision,k=d.act.k;
  $('hSpd').textContent=(ego.v*3.6).toFixed(0);
  const ha=$('hAct');ha.textContent=d.act.label.toUpperCase();ha.className='act '+actClass(k);
  $('hAcc').textContent=ego.acc.toFixed(1)+' m/s²';$('hObj').textContent=d.tracks.length;
  $('hRisk').textContent=Math.round(d.maxNo*100)+'%';$('hRisk').style.color=riskCol(d.maxNo);
  $('hLat').textContent=d.lat.toFixed(2)+' ms';
  $('tSpd').textContent=(ego.v*3.6).toFixed(0)+' km/h';
  const ttcs=d.tracks.filter(t=>t.ttc!=null).map(t=>t.ttc);$('tTtc').textContent=ttcs.length?Math.min(...ttcs).toFixed(1)+' s':'—';
  $('tTtc').style.color=ttcs.length&&Math.min(...ttcs)<2.5?'#ff4d5e':'';
  $('tTrips').textContent=stats.trips;
  updateRideUI();updatePidUI();updateTrafficUI();
  $('hNav').innerHTML=navText();
  if(route){const r=route,rem=Math.max(0,(r.n-1-ego.pi)*DS),dx=r.px[r.n-1]-ego.x,dy=r.py[r.n-1]-ego.y,co=Math.cos(ego.th),si=Math.sin(ego.th);
    const brg=Math.atan2(-dx*si+dy*co,dx*co+dy*si)*180/Math.PI,turns=r.nodes.filter(n=>n.turn&&n.q>ego.pi).length,cl=ctrlCount(r,ego.pi),m=PY.etaModel,eta=m?Math.max(0,m.per_m*rem+m.per_turn*turns+(m.per_ctrl||0)*cl+m.base_s*rem/Math.max(r.len,1)):rem/7+turns*4+cl*6;
    $('dName').textContent=(r.kind?r.kind+': ':'')+r.name;$('dPct').textContent=Math.round(100*(1-rem/r.len))+'%';$('dProg').style.width=(100*(1-rem/r.len)).toFixed(1)+'%';
    $('dRem').textContent=rem.toFixed(0)+' m';$('dEuc').textContent=Math.hypot(dx,dy).toFixed(0)+' m';
    $('dBrg').textContent=Math.abs(brg)<4?'ahead':Math.abs(brg)>176?'behind':Math.abs(brg).toFixed(0)+'° '+(brg>0?'R':'L');
    $('dEta').textContent=(ride.phase==='boarding'||ride.phase==='alighting'||ride.phase==='idle')?'—':Math.floor(eta/60)+':'+String(Math.floor(eta%60)).padStart(2,'0');
    $('dTurns').textContent=turns;$('dTot').textContent=r.len.toFixed(0)+' m';}
  $('dLoc').innerHTML=locText();
  $('oTot').textContent=odo.total>1000?(odo.total/1000).toFixed(2)+' km':odo.total.toFixed(0)+' m';
  $('oTrip').textContent=odo.trip.toFixed(0)+' m';$('oYaw').textContent=((ego.omega||0)*180/Math.PI).toFixed(1)+'°/s';
  {const e=odo.fus.x-odo.start.x,n=-(odo.fus.y-odo.start.y),hdg=((odo.fus.th*180/Math.PI+90)%360+360)%360;
   $('oPose').textContent=`E ${e>=0?'+':''}${e.toFixed(1)} m · N ${n>=0?'+':''}${n.toFixed(1)} m · ${hdg.toFixed(0).padStart(3,'0')}°`;}
  $('oRaw').textContent=odo.err.raw.toFixed(2)+' m';$('oFus').textContent=odo.err.fus.toFixed(2)+' m';$('oFix').textContent=(simT-odo.fixT).toFixed(1)+' s ago';
  if(ego.pp){const pp=ego.pp;$('pLd').textContent=pp.Ld.toFixed(1)+' m';$('pSteer').textContent=(ego.steer*180/Math.PI).toFixed(1)+'°';
    $('pRad').textContent=Math.abs(pp.kappa)<.002?'straight':(1/Math.abs(pp.kappa)).toFixed(1)+' m';$('pAlpha').textContent=(pp.alpha*180/Math.PI).toFixed(1)+'°';
    $('pCte').textContent=(ego.cte>=0?'+':'')+ego.cte.toFixed(2)+' m';$('pHe').textContent=(ego.headErr*180/Math.PI).toFixed(1)+'°';}
  {const g={Vehicle:['car'],Pedestrian:['ped','child'],Animal:['dog','cat','deer']};let h='';
   for(const[k,ks]of Object.entries(g)){const t=decision.tracks.filter(x=>ks.includes(x.o.kind)).sort((a,b)=>a.dist-b.dist)[0];
     h+=`<div><span>Nearest ${k.toLowerCase()}</span><span class="mono">${t?`${t.dist.toFixed(1)} m · ${fmtRel(t.fwd,t.lat)}`:'none in range'}</span></div>`;}
   if(route){const r=route,dx=r.px[r.n-1]-ego.x,dy=r.py[r.n-1]-ego.y,co=Math.cos(ego.th),si=Math.sin(ego.th);
     h+=`<div><span>Destination</span><span class="mono">${Math.hypot(dx,dy).toFixed(0)} m · ${fmtRel(dx*co+dy*si,-dx*si+dy*co)}</span></div>`;}
   h+=`<div><span>Tracked total</span><span class="mono">${decision.tracks.length} objects</span></div>`;
   $('near').innerHTML=h;}
  $('tEm').textContent=stats.em;$('tCol').textContent=stats.col;$('tCol').style.color=stats.col?'#ff4d5e':'';
  const avg=latHist.reduce((a,b)=>a+b,0)/Math.max(1,latHist.length);$('tLat').textContent=avg.toFixed(2)+' ms';
  $('dTick').textContent=`${d.results.length} plans × ${d.tracks.length} tracks × ${NS} steps`;
  const maxC=Math.max(...d.results.map(r=>r.cost),1);
  $('actions').innerHTML=d.results.map(r=>{const sel=r.act===d.act;return `<div class="arow ${sel?'sel':''}"><span class="nm" style="color:${sel?'#38e1c6':''}">${sel?'▸ ':''}${r.act.label}</span><span class="bar"><i style="width:${(r.cost/maxC*100).toFixed(1)}%;background:${riskCol(r.Rmax)}"></i></span><span class="num" style="color:${riskCol(r.Rmax)}">${Math.round(r.Rmax*100)}%</span><span class="num">${r.cost.toFixed(1)}</span></div>`;}).join('');
  const top=d.tracks[0];
  let why;
  if(k==='CRUISE')why=top&&top.pNo>.1?`Monitoring ${CLSNAME[top.o.kind].toLowerCase()} #${top.o.id} (P ${Math.round(top.pNo*100)}%) — cruise is still the cheapest safe plan.`:'Path clear — nominal driving (IDM car-following, '+((ego.vlim||VLIM)*3.6).toFixed(0)+' km/h limit'+(ego.ctl&&ego.ctl.hold?', holding for '+({red:'a red light',amber:'an amber light',left:'oncoming traffic',stop:'a stop sign',queue:'its turn at the all-way stop',gap:'a gap in traffic',yield:'the yield sign'}[ego.ctl.why]||'traffic control'):'')+').';
  else why=`<b>${d.act.label}</b> chosen: ${top?`${CLSNAME[top.o.kind].toLowerCase()} #${top.o.id} at ${top.dist.toFixed(1)} m, P(collision) ${Math.round(top.pNo*100)}% → ${Math.round(top.pPlan*100)}% with this plan${top.ttc!=null?`, TTC ${top.ttc.toFixed(1)} s`:''}.`:'reducing residual risk.'}`;
  $('why').innerHTML=why;$('why').style.borderLeftColor=k==='CRUISE'?'#38e1c6':k==='EMERGENCY'?'#ff4d5e':'#ffb020';
  $('nTrk').textContent=d.tracks.length;
  $('objs').innerHTML=d.tracks.slice(0,10).map(t=>`<tr><td><span class="chip" style="background:${CCOL[t.o.kind]}">${CLS[t.o.kind]}</span> <span style="color:#7c90a3">#${t.o.id}</span></td><td>${t.dist.toFixed(1)}</td><td style="font-size:10.5px">${fmtRel(t.fwd,t.lat)}</td><td style="color:${t.closing>1?'#ffb020':'#7c90a3'}">${t.closing>=0?'+':''}${t.closing.toFixed(1)}</td><td style="color:${t.ttc!=null&&t.ttc<2.5?'#ff4d5e':''}">${t.ttc!=null?t.ttc.toFixed(1)+'s':'—'}</td><td><span style="color:${riskCol(t.pPlan)}">${Math.round(t.pPlan*100)}%</span><span style="color:#566a7c;font-size:10px"> /${Math.round(t.pNo*100)}</span></td></tr>`).join('')||'<tr><td colspan="6" style="color:#7c90a3;text-align:left">No objects in range</td></tr>';
  const cr=$('crunch');
  if(d.crit&&k!=='CRUISE'){cr.classList.add('show');$('cTitle').textContent='⚠ CRUNCH TIME — '+CLSNAME[d.crit.o.kind].toUpperCase();
    $('cBody').textContent=`P(collision) ${Math.round(d.crit.pNo*100)}%${d.crit.ttc!=null?' · TTC '+d.crit.ttc.toFixed(1)+' s':''} → ${d.act.label.toUpperCase()} · decided in ${d.lat.toFixed(2)} ms`;}
  else cr.classList.remove('show');
  const t=simT;$('simClock').textContent=String(Math.floor(t/60)).padStart(2,'0')+':'+String(Math.floor(t%60)).padStart(2,'0');
}
function setupUI(){
  $('scens').innerHTML=SCEN.map((s,i)=>`<button class="sc" data-i="${i}"><kbd>${i+1}</kbd>${s.icon} ${s.name}</button>`).join('');
  $('scens').addEventListener('click',e=>{const b=e.target.closest('button');if(b)runScenario(+b.dataset.i);});
  const tog=(id,fn)=>$(id).addEventListener('click',()=>{const on=!$(id).classList.contains('on');$(id).classList.toggle('on',on);fn(on);});
  tog('tgChaos',v=>{autoChaos=v;nextChaos=simT+3;});tog('tgSlow',v=>autoSlow=v);
  tog('tgHead',v=>headingUp=v);tog('tgTilt',v=>{$('tilt').classList.toggle('on',v);});
  tog('tgPred',v=>showPred=v);tog('tgRoute',v=>showRoute=v);tog('tgOdo',v=>showOdo=v);tog('tgDist',v=>showDist=v);
  $('reqRide').onclick=()=>openBooking();$('bookTop').onclick=()=>openBooking();setupBooking();
  tog('tgAuto',v=>ride.auto=v);tog('tgFF',v=>{ctl.ffOn=v;ctl.I=0;});
  $('zRules').addEventListener('click',e=>{const tr=e.target.closest('tr');if(!tr)return;ctl.rule=tr.dataset.r;ctl.znT=-1;ctl.I=0;logE(`PID tuning rule → ${RULES[ctl.rule].name}`,'info');});
  $('zCsv').onclick=async()=>{const rows=['t,x,y,v,mu,grade,mode,mass,K,tau,Ku,Tu,Kp,Ki,Kd'].concat(znLog.map(r=>[r.t.toFixed(1),r.x.toFixed(2),r.y.toFixed(2),r.v.toFixed(2),r.mu,r.gr.toFixed(4),r.mode,r.m,r.K.toFixed(1),r.tau.toFixed(1),r.Ku.toFixed(4),r.Tu.toFixed(4),r.Kp.toFixed(4),r.Ki.toFixed(4),r.Kd.toFixed(5)].join(',')));
    try{await navigator.clipboard.writeText(rows.join('\n'));$('zCsv').textContent='Copied '+znLog.length+' rows';}catch(err){$('zCsv').textContent='Copy blocked';}setTimeout(()=>$('zCsv').textContent='Copy CSV',2000);};
  $('miniWrap').addEventListener('click',()=>openBooking());
  $('miniWrap').title='Click to book a ride';tog('tgLidar',v=>showLidar=v);tog('tgPause',v=>paused=v);
  $('speeds').innerHTML=[.25,.5,1,2].map(s=>`<button data-s="${s}" class="${s===1?'on':''}">${s}×</button>`).join('');
  $('speeds').addEventListener('click',e=>{const b=e.target.closest('button');if(!b)return;simSpeed=+b.dataset.s;[...$('speeds').children].forEach(c=>c.classList.toggle('on',c===b));});
  $('zIn').onclick=()=>Z=clamp(Z*1.25,3,30);$('zOut').onclick=()=>Z=clamp(Z/1.25,3,30);
  $('stage').addEventListener('wheel',e=>{e.preventDefault();Z=clamp(Z*Math.exp(-e.deltaY*.0015),3,30);},{passive:false});
  window.addEventListener('keydown',e=>{
    if(e.target.tagName==='INPUT'||bk.open)return;
    if(e.key>='1'&&e.key<='6'){runScenario(+e.key-1);}
    else if(e.key===' '){e.preventDefault();$('tgPause').click();}
    else if(e.key==='h'||e.key==='H')$('tgHead').click();
    else if(e.key==='t'||e.key==='T')$('tgTilt').click();
    else if(e.key==='+'||e.key==='=')Z=clamp(Z*1.25,3,30);
    else if(e.key==='-')Z=clamp(Z/1.25,3,30);
  });
}
let last=0,lastUI=0;
function frame(now){
  const dt=Math.min(.05,(now-last)/1000||0);last=now;
  const tgt=autoSlow&&decision.crit&&decision.act.k!=='CRUISE'?.3:1;slowF+=(tgt-slowF)*Math.min(1,dt*5);
  if(!paused){let sdt=dt*simSpeed*slowF;while(sdt>1e-6){const h=Math.min(sdt,1/60);step(h);sdt-=h;}}
  render(dt);
  if(now-lastUI>110){updateUI();lastUI=now;}
  requestAnimationFrame(frame);
}
// ============================== Python back-end (Flask) ==============================
async function connectPython(){
  try{const m=await api('/api/map',null,2500);PY.map=m;PY.online=true;
    HNAME.splice(0,HNAME.length,...m.hnames);VNAME.splice(0,VNAME.length,...m.vnames);   // street names from the BeautifulSoup-parsed XML
    VLIM=m.speed_limit_ms;Object.assign(VEH,m.vehicle);}
  catch(e){PY.online=false;}
  updatePyPill();
}
function updatePyPill(){const p=$('pyPill');if(!p)return;p.innerHTML=PY.online?'python <b style="color:#3ddc84">connected</b>':'python <b style="color:#ffb020">offline</b>';}
async function pushToPython(){
  if(!PY.online)return;const batch={session:PY.session};let any=false;
  for(const k in OUT){if(OUT[k].length){batch[k]=OUT[k].splice(0,OUT[k].length);any=true;}}
  if(!any)return;
  try{await api('/api/ingest',batch);}catch(e){for(const k in OUT)if(batch[k])OUT[k].unshift(...batch[k]);}
}
async function pullAnalytics(){
  if(!PY.online)return;
  try{const a=await api('/api/analytics');PY.analytics=a;PY.etaModel=a.eta;
    const f=a.rf_fit;
    if(PY.useRfFit&&f&&f.samples>=150){RF_MODEL.P0=f.P0;RF_MODEL.n=f.n;RF_MODEL.src=`scikit-learn fit (${f.samples} samples)`;}
    else if(!PY.useRfFit){RF_MODEL.P0=-45;RF_MODEL.n=2.0;RF_MODEL.src='generic BLE default';}
    updateAnalyticsUI();}catch(e){}
}
const f2=(v,d=2)=>v==null?'—':(+v).toFixed(d);
function updateAnalyticsUI(){
  const a=PY.analytics,box=$('pyBody');if(!box)return;
  if(!PY.online){box.innerHTML='<div class="note">Offline mode: the page was opened without the Python server, so the simulation runs with built-in defaults and no analytics. For the full version, double-click <b>Start Robotaxi.command</b> (Mac) or <b>start_windows.bat</b> (Windows), or run <b>python3 app.py</b>, then use http://127.0.0.1:5000.</div>';return;}
  if(!a){box.innerHTML='<div class="note">Waiting for data…</div>';return;}
  const s=a.summary||{},f=a.rf_fit,e=a.eta;
  let h=`<div class="tiles">
    <div class="tile"><div class="k">Pickups</div><div class="v">${s.pickups||0}</div></div>
    <div class="tile"><div class="k">Door · RF</div><div class="v" style="color:#38e1c6">${s.door_rf_mean_m!=null?f2(s.door_rf_mean_m,1)+' m':'—'}</div></div>
    <div class="tile"><div class="k">Door · GPS</div><div class="v" style="color:#ff8a3d">${s.door_gps_mean_m!=null?f2(s.door_gps_mean_m,1)+' m':'—'}</div></div>
    <div class="tile"><div class="k">Speed RMS err</div><div class="v">${f2(s.speed_rms_error_ms)} m/s</div></div>
    <div class="tile"><div class="k">Planner p95</div><div class="v">${f2(s.planner_ms_p95)} ms</div></div>
    <div class="tile"><div class="k">Rows logged</div><div class="v">${(a.counts.telemetry||0)+(a.counts.rf||0)}</div></div></div>`;
  h+=`<h3 style="margin-top:12px">Radio model · LinearRegression</h3>`+(f?`<div class="note" style="margin-top:0">RSSI = P0 − 10·n·log₁₀(d), fitted on ${f.samples} BLE samples: <b class="mono">P0 ${f.P0} dBm, n ${f.n}</b>, R² ${f.r2}, noise ${f.shadow_db} dB · truth P0 −41, n 2.3. The particle filter is using: <b>${RF_MODEL.src}</b>.</div>`
      :`<div class="note" style="margin-top:0">Collecting BLE samples (needs 40+). Until then the filter uses the generic model P0 −45, n 2.0.</div>`);
  h+=`<h3 style="margin-top:12px">Trip-time model · LinearRegression</h3>`+(e?`<div class="note" style="margin-top:0">duration ≈ <b class="mono">${e.base_s} s + ${e.per_m} s/m · length + ${e.per_turn} s/turn${e.per_ctrl!=null?' + '+e.per_ctrl+' s per controlled intersection':''}</b> · MAE ${e.mae_s} s on ${e.samples} legs${e.r2!=null?', R² '+e.r2:''}. Used for every ETA on the page.</div>`
      :`<div class="note" style="margin-top:0">Needs 3+ completed legs; ETAs use a heuristic until then.</div>`);
  h+=`<h3 style="margin-top:12px">Risk hotspots · DBSCAN</h3>`+(a.hotspots.length?a.hotspots.map((x,i)=>`<div class="note" style="margin-top:2px">${i+1}. <b>${x.where}</b> — ${x.count} high-risk moments, mostly ${x.cause.toLowerCase()}</div>`).join(''):'<div class="note" style="margin-top:0">No clusters yet — they appear as red dashed circles on the map.</div>');
  if(a.zn&&a.zn.length)h+=`<h3 style="margin-top:12px">Ziegler–Nichols by condition · pandas groupby</h3><table><thead><tr><th>Mode</th><th>Surface</th><th>Points</th><th>Ku</th><th>Tu</th><th>Kp</th></tr></thead><tbody>`+a.zn.map(r=>`<tr><td>${r.mode}</td><td>${r.surface}</td><td>${r.points}</td><td>${f2(r.Ku_mean)}</td><td>${f2(r.Tu_mean)}</td><td>${f2(r.Kp_mean)}</td></tr>`).join('')+'</tbody></table>';
  box.innerHTML=h;
}
function setupPythonUI(){
  const t=(id,fn)=>{const b=$(id);if(b)b.addEventListener('click',()=>{const on=!b.classList.contains('on');b.classList.toggle('on',on);fn(on);});};
  t('tgRfFit',v=>{PY.useRfFit=v;pullAnalytics();});t('tgHot',v=>PY.showHot=v);
  setInterval(pushToPython,2000);setInterval(pullAnalytics,4000);updateAnalyticsUI();
}

setTimeout(async()=>{
  await connectPython();
  setupTraffic();const tg=buildTexture();initWorld(tg);setupUI();ctl.vref=ego.v;ctl.vPrev=ego.v;makeRequest();ego.nominal=egoNominal();resize();plan();setupPythonUI();
  $('loading').style.display='none';
  logE('Robotaxi online — perception, prediction and planning running at 10 Hz','info');
  requestAnimationFrame(t=>{last=t;requestAnimationFrame(frame);});
},30);

// ============================== concept pages (left navigation) ==============================
const PAGE_IDS=[...document.querySelectorAll('#pnav a')].map(a=>a.dataset.page);
function showPage(id){
  if(!PAGE_IDS.includes(id))id='overview';
  document.querySelectorAll('aside [data-page]').forEach(el=>el.classList.toggle('show',el.dataset.page===id));
  document.querySelectorAll('#pnav a').forEach(a=>a.classList.toggle('on',a.dataset.page===id));
  document.querySelector('aside').scrollTop=0;
  if(id==='log'&&typeof renderLog==='function')renderLog();
}
window.addEventListener('hashchange',()=>showPage(location.hash.slice(1)));
showPage(location.hash.slice(1)||'overview');
