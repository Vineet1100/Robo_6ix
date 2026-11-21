/* =============================================================================
 * 01-config.js — Constants & helpers
 * Map size, speed limit, lane geometry, colours per object class, object sizes, risk model σ values, detection confidence, small maths helpers.
 * Loaded as a classic <script>; all files share one global scope, in the order
 * listed in index.html.
 * ============================================================================= */
'use strict';
// ============================== constants & utils ==============================
const P=60,N=5,W=P*N,M=40,RH=4,LO=2,SW=6.5,NODE=5.2,TS=8,RANGE=60;
let VLIM=11.1; // 40 km/h downtown limit (replaced by the speed limit in the Python map file when the server is running)

// ---------------- Python (Flask) back-end bridge ----------------
// The page still works if opened on its own (offline mode) — it then uses the built-in defaults.
const PY={online:false,map:null,analytics:null,etaModel:null,useRfFit:true,showHot:true,session:Math.random().toString(36).slice(2)};
const OUT={telemetry:[],events:[],pickups:[],trips:[],rf:[],zn_log:[]};   // batches streamed to Python every 2 s
async function api(path,body,timeout=4000){const ctl=new AbortController(),tm=setTimeout(()=>ctl.abort(),timeout);
  try{const r=await fetch(path,body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:ctl.signal}:{signal:ctl.signal});
    if(!r.ok)throw new Error('HTTP '+r.status);return await r.json();}finally{clearTimeout(tm);}}
const DIR=[[1,0],[0,1],[-1,0],[0,-1]];
const rightV=d=>DIR[(d+1)%4];
const sgnD=d=>(d===0||d===1)?1:-1;
const horiz=d=>d%2===0;
const clamp=(v,a,b)=>v<a?a:v>b?b:v;
function angDiff(a,b){let d=b-a;while(d>Math.PI)d-=2*Math.PI;while(d<-Math.PI)d+=2*Math.PI;return d;}
function mulberry(seed){return()=>{seed|=0;seed=seed+0x6D2B79F5|0;let t=Math.imul(seed^seed>>>15,1|seed);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};}
const SR=mulberry(20261112);
const sr=(a,b)=>a+(b-a)*SR();
const rnd=(a,b)=>a+(b-a)*Math.random();
const pick=(arr,r=Math.random)=>arr[Math.floor(r()*arr.length)];
function rr(c,x,y,w,h,r){r=Math.min(r,w/2,h/2);c.beginPath();c.moveTo(x+r,y);c.arcTo(x+w,y,x+w,y+h,r);c.arcTo(x+w,y+h,x,y+h,r);c.arcTo(x,y+h,x,y,r);c.arcTo(x,y,x+w,y,r);c.closePath();}
function shade(hex,f){const n=parseInt(hex.slice(1),16);let r=n>>16,g=n>>8&255,b=n&255;
  if(f>0){r+=(255-r)*f;g+=(255-g)*f;b+=(255-b)*f;}else{r*=1+f;g*=1+f;b*=1+f;}
  return `rgb(${r|0},${g|0},${b|0})`;}
function onRoad(x,y){if(x<-RH||y<-RH||x>W+RH||y>W+RH)return false;
  return Math.abs(x-Math.round(x/P)*P)<RH||Math.abs(y-Math.round(y/P)*P)<RH;}
const laneCoord=(d,rc)=>rc+LO*rightV(d)[horiz(d)?1:0];

const CLS={car:'VEH',ped:'PED',child:'CHILD',dog:'DOG',cat:'CAT',deer:'DEER',raccoon:'RACC',ball:'BALL'};
const CLSNAME={car:'Vehicle',ped:'Pedestrian',child:'Child',dog:'Dog',cat:'Cat',deer:'Deer',raccoon:'Raccoon',ball:'Ball / debris'};
const CCOL={car:'#4da3ff',ped:'#ffd84d',child:'#ffe98a',dog:'#ff8a3d',cat:'#ff8a3d',deer:'#ff8a3d',raccoon:'#ff8a3d',ball:'#c792ff'};
const SIZE={car:{r:.95,b:0},ped:{r:.35,b:.95},child:{r:.3,b:.75},dog:{r:.45,b:1.15},cat:{r:.25,b:.65},deer:{r:.75,b:2.0},raccoon:{r:.35,b:1.0},ball:{r:.15,b:.5}};
const SIG={car:{s0:.25,kr:.12,ks:.12},ped:{s0:.3,kr:.35,ks:.1},child:{s0:.3,kr:.55,ks:.18},dog:{s0:.35,kr:.7,ks:.2},cat:{s0:.3,kr:.6,ks:.2},deer:{s0:.45,kr:.65,ks:.3},raccoon:{s0:.35,kr:.7,ks:.25},ball:{s0:.15,kr:.3,ks:.3}};
const CONF0={car:.985,ped:.955,child:.91,dog:.9,cat:.82,deer:.93,raccoon:.88,ball:.76};
const CARCOLS=['#d8d8d8','#1d1f22','#8a8f96','#a12a2a','#24467a','#e7e3da','#3b4f3a','#c7a43a','#5a2e4f','#f0f0f0','#4a4d52'];
const SHIRTS=['#d9534f','#3b7dd8','#f2f2f2','#2d2d2d','#e2b13c','#5cb85c','#9b59b6','#e67e22','#1abc9c'];
const riskCol=p=>p<.1?'#3ddc84':p<.35?'#ffb020':'#ff4d5e';

