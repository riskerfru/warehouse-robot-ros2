import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import os
from http.server import HTTPServer, BaseHTTPRequestHandler

STATE = {
    "robot_position":    {"x": 0.0, "y": 0.0},
    "status":            "Virtual Warehouse Ready",
    "task_log":          [],
    "waypoints":         [],
    "current_step":      0,
    "total_steps":       0,
    "holding":           False,
    "planned_path":      [],
    "dynamic_obstacles": [],
    "target_color":      None,
    "world_bins":        [],
    "remove_bin_id":     None,
    "rrt_tree":          {"edges":[],"obstacles":[],"radius":0.35,"nodes":0},
    "lidar":             {"hits":[],"ranges":[],"fov":270,"max_range":3.5,"sensed":0},
}

HTML = r'''<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>AI Warehouse Robot 3D</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:monospace;background:#1a1a1a;color:#e0e0e0;height:100vh;display:grid;grid-template-rows:48px 1fr 130px;overflow:hidden}
header{background:#111;border-bottom:1px solid #333;display:flex;align-items:center;padding:0 16px;gap:10px}
header h1{font-size:14px;font-weight:700;color:#fff}
.badge{padding:2px 8px;border-radius:12px;font-size:10px;font-weight:700}
.online{background:#0c6;color:#000}.offline{background:#c33;color:#fff}
.estop{margin-left:auto;background:#c00;color:#fff;border:none;padding:5px 14px;border-radius:3px;font-weight:700;cursor:pointer;font-size:12px}
.main{display:grid;grid-template-columns:1fr 190px;overflow:hidden;min-height:0}
#stage{display:grid;grid-template-columns:1fr 1fr;overflow:hidden;height:100%}
#sw{position:relative;background:#111;overflow:hidden}
#schem{position:relative;background:#0b0e12;border-left:1px solid #232a33;overflow:hidden}
#sc{position:absolute;inset:0;width:100%;height:100%}
.schead{position:absolute;top:0;left:0;right:0;height:26px;background:#0e1319;border-bottom:1px solid #232a33;
  display:flex;align-items:center;padding:0 10px;gap:10px;font-size:9px;letter-spacing:1.6px;
  color:#5d6b7a;text-transform:uppercase;z-index:4}
.schead b{color:#7fb3d5;font-weight:600;letter-spacing:1.6px}
.scstat{position:absolute;bottom:0;left:0;right:0;height:58px;background:#0e1319;border-top:1px solid #232a33;
  display:grid;grid-template-columns:repeat(4,1fr);z-index:4}
.scell{padding:6px 8px;border-right:1px solid #1a2028}
.scell:last-child{border-right:none}
.sck{font-size:8px;color:#4d5966;text-transform:uppercase;letter-spacing:1.2px}
.scv{font-size:13px;color:#8fd3c7;font-weight:600;margin-top:2px}
#tc{position:absolute;top:0;left:0;width:100%;height:100%}
.ov{position:absolute;top:8px;left:8px;background:rgba(0,0,0,0.8);border:1px solid #444;border-radius:4px;padding:5px 9px;font-size:11px;color:#4af;pointer-events:none;z-index:5}
.hint{position:absolute;bottom:6px;left:8px;font-size:9px;color:#555;pointer-events:none;z-index:5}
.sp{background:#111;border-left:1px solid #2a2a2a;padding:8px;display:flex;flex-direction:column;gap:6px;overflow-y:auto}
.sp h3{font-size:8px;color:#555;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:2px}
.sb{background:#181818;border-radius:4px;padding:7px;border:1px solid #222}
.sr{display:flex;justify-content:space-between;padding:1px 0;font-size:10px;border-bottom:1px solid #0d0d0d}
.sr:last-child{border:none}
.sl{color:#555}.sv{color:#4af;font-weight:700}.g{color:#4f4}
.pw{background:#0a0a0a;border-radius:2px;height:4px;overflow:hidden;margin-top:3px}
.pb{height:100%;background:linear-gradient(90deg,#4a9eff,#0fa);border-radius:2px;transition:width 0.4s}
.zi{padding:3px 5px;margin:1px 0;border-radius:2px;cursor:pointer;font-size:10px;border:1px solid transparent;transition:all 0.15s}
.zi:hover{border-color:#4af;background:#1a2030}
.zs{background:#0d1520;color:#4af}.zp{background:#0d1a0d;color:#4f4}
.zd{background:#1a0d0d;color:#f44}.zh{background:#0d0d1a;color:#aaf}
.bb{background:#111;border-top:1px solid #2a2a2a;display:grid;grid-template-columns:1fr 220px;padding:8px 12px;gap:10px;align-items:start}
.os h3{font-size:8px;color:#555;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:5px}
.or{display:flex;gap:5px;margin-bottom:5px}
.or input{flex:1;background:#181818;border:1px solid #333;color:#fff;padding:6px 8px;border-radius:3px;font-size:11px}
.or input:focus{outline:none;border-color:#4af}
.sb2{background:#4a9eff;color:#fff;border:none;padding:6px 12px;border-radius:3px;cursor:pointer;font-weight:700;font-size:11px}
.qr{display:flex;gap:3px;flex-wrap:wrap}
.qb{background:#181818;border:1px solid #2a2a2a;color:#777;padding:2px 7px;border-radius:2px;cursor:pointer;font-size:9px}
.qb:hover{border-color:#4af;color:#4af}
.ls h3{font-size:8px;color:#555;text-transform:uppercase;letter-spacing:1.5px;margin-bottom:3px}
.lb{height:68px;overflow-y:auto;font-size:9px}
.le{padding:1px 0;color:#555}
.le.i{color:#4af}.le.o{color:#4f4}.le.w{color:#fa4}.le.e{color:#f44}
</style>
</head>
<body>
<header>
  <h1>AI WAREHOUSE ROBOT / ROS2</h1>
  <span class="badge online" id="badge">ROS2</span>
  <span id="stxt" style="font-size:10px;color:#666">Virtual Warehouse Ready</span>
  <button class="estop" onclick="eStop()">E-STOP</button>
</header>
<div class="main">
  <div id="stage">
  <div id="sw">
    <canvas id="tc"></canvas>
    <div class="ov">X:<span id="ox">0.00</span>m &nbsp; Y:<span id="oy">0.00</span>m &nbsp; Step:<span id="ostp">-</span></div>
    <div class="hint">Drag orbit · Scroll zoom · Shelves are solid obstacles · RRT* avoids them</div>
  </div>
  <div id="schem">
    <div class="schead"><b>PLANNER SCHEMATIC</b><span>RRT* / TOPP-RA</span></div>
    <canvas id="sc"></canvas>
    <div class="scstat">
      <div class="scell"><div class="sck">Tree nodes</div><div class="scv" id="scNodes">0</div></div>
      <div class="scell"><div class="sck">Lidar hits</div><div class="scv" id="scPts">0</div></div>
      <div class="scell"><div class="sck">Inflation</div><div class="scv" id="scInf">0.35 m</div></div>
      <div class="scell"><div class="sck">State</div><div class="scv" id="scState">IDLE</div></div>
    </div>
  </div>
  </div>
  <div class="sp">
    <div class="sb">
      <h3>Robot</h3>
      <div class="sr"><span class="sl">X</span><span class="sv" id="px">0.00m</span></div>
      <div class="sr"><span class="sl">Y</span><span class="sv" id="py">0.00m</span></div>
      <div class="sr"><span class="sl">Holding</span><span class="sv" id="hld">Nothing</span></div>
      <div class="sr"><span class="sl">Step</span><span class="sv" id="stp">-</span></div>
      <div class="pw"><div class="pb" id="prg" style="width:0%"></div></div>
    </div>
    <div>
      <h3>Quick Orders</h3>
      <div class="zi zs" onclick="sendQ('pick red bin from shelf A and deliver to dispatch')">Red - Shelf A</div>
      <div class="zi zs" onclick="sendQ('pick blue bin from shelf B and deliver to dispatch')">Blue - Shelf B</div>
      <div class="zi zs" onclick="sendQ('pick green bin from shelf C and deliver to dispatch')">Green - Shelf C</div>
      <div class="zi zs" onclick="sendQ('pick orange bin from shelf B and deliver to dispatch')">Orange - Shelf B</div>
      <div class="zi zp" onclick="sendQ('go to pick zone')">Pick Zone</div>
      <div class="zi zd" onclick="sendQ('go to dispatch zone')">Dispatch</div>
      <div class="zi zh" onclick="sendQ('return to home position')">Home</div>
    </div>
    <div class="sb">
      <h3>System</h3>
      <div class="sr"><span class="sl">ROS2</span><span class="sv g">Active</span></div>
      <div class="sr"><span class="sl">RRT*</span><span class="sv g">Ready</span></div>
      <div class="sr"><span class="sl">TOPP-RA</span><span class="sv g">Ready</span></div>
      <div class="sr"><span class="sl">Claude AI</span><span class="sv g">Ready</span></div>
    </div>
  </div>
</div>
<div class="bb">
  <div class="os">
    <h3>Mission Control</h3>
    <div class="or">
      <input type="text" id="oi" placeholder="pick red bin from shelf A and deliver to dispatch..." onkeydown="if(event.key==='Enter')sendOrder()"/>
      <button class="sb2" onclick="sendOrder()">▶</button>
    </div>
    <div class="qr">
      <span class="qb" onclick="q('pick red bin from shelf A and deliver to dispatch')">Red A</span>
      <span class="qb" onclick="q('pick blue bin from shelf B and deliver to dispatch')">Blue B</span>
      <span class="qb" onclick="q('pick green bin from shelf C and deliver to dispatch')">Green C</span>
      <span class="qb" onclick="q('pick yellow bin from shelf A and deliver to dispatch')">Yellow A</span>
      <span class="qb" onclick="q('pick orange bin from shelf B and deliver to dispatch')">Orange B</span>
      <span class="qb" onclick="q('return to home position')">Home</span>
    </div>
  </div>
  <div class="ls">
    <h3>Task Log</h3>
    <div class="lb" id="lb"></div>
  </div>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
// ── State ────────────────────────────────────────────────────
let state = {};
let robotGroup = null, beaconLight = null, statusLight = null;
let armJ = [];
let wheelMeshes = [], wheelAngle = 0;
let armTarget = 0, armCurrent = 0;
let robotX = -6, robotZ = -5;
let rrtPath = [], rrtIdx = 0, followPath = false;
let rrtLine = null;
let shelfBins = [];   // {mesh, color, shelfX, z, visible}
let dynObsMeshes = [];
let heldBin = null;

// Layout
const SHELF_X     = [-3.0, 0.0, 3.0];
const SHELF_NAMES = ['A', 'B', 'C'];
const HOME        = {x:-6, z:-5};
const PICK        = {x:-6, z: 5};
const DISPATCH    = {x: 6, z: 5};
const BIN_COLORS  = [0x9c3229,0x2f5488,0x3f7a45,0xc2962c,0x6b4a7a,0xc06424,0x3f8894];
const BIN_NAMES   = ['red','blue','green','yellow','purple','orange','cyan'];

// ── Three.js ─────────────────────────────────────────────────
const wrap = document.getElementById('sw');
const renderer = new THREE.WebGLRenderer({canvas:document.getElementById('tc'),antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.setClearColor(0x8e959c,1);   // light grey like real warehouse
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.95;

const scene  = new THREE.Scene();
scene.fog = new THREE.Fog(0x8e959c, 16, 44);
const camera = new THREE.PerspectiveCamera(48,1,0.1,100);

// Robot's onboard camera — used for REAL vision detection
const robotCam = new THREE.PerspectiveCamera(60, 1.6, 0.1, 30);
let lastScanSent = 0;

function resize(){
  const w=wrap.clientWidth,h=wrap.clientHeight;
  if(!w||!h)return;
  renderer.setSize(w,h,false);
  camera.aspect=w/h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize',resize);

// ── Lighting (bright warehouse fluorescent) ──────────────────
scene.add(new THREE.AmbientLight(0xc9d2dc, 0.42));

const sun = new THREE.DirectionalLight(0xffe9c4, 0.95);
sun.position.set(5,18,5);
sun.castShadow = true;
sun.shadow.mapSize.set(2048,2048);
sun.shadow.camera.left = sun.shadow.camera.bottom = -14;
sun.shadow.camera.right = sun.shadow.camera.top = 14;
scene.add(sun);

// Fluorescent tube lights above aisles
[-6,-1.5,1.5,6].forEach(ax=>{
  const pl = new THREE.RectAreaLight(0xfff8f0,8,1,8);
  pl.position.set(ax,5.5,0);
  pl.rotation.x = -Math.PI/2;
  scene.add(pl);
});

// Orbit controls
let drag=false, pm={x:0,y:0}, sph={theta:0.15, phi:0.52, r:19};
renderer.domElement.addEventListener('mousedown',e=>{drag=true;pm={x:e.clientX,y:e.clientY};});
window.addEventListener('mouseup',()=>drag=false);
window.addEventListener('mousemove',e=>{
  if(!drag)return;
  sph.theta-=(e.clientX-pm.x)*0.005;
  sph.phi=Math.max(0.12,Math.min(1.3,sph.phi+(e.clientY-pm.y)*0.005));
  pm={x:e.clientX,y:e.clientY};
});
renderer.domElement.addEventListener('wheel',e=>{
  sph.r=Math.max(5,Math.min(28,sph.r+e.deltaY*0.01));
},{passive:true});

// ── Materials ────────────────────────────────────────────────
const concreteMat  = new THREE.MeshLambertMaterial({color:0xb8bec6});
const steelMat     = new THREE.MeshLambertMaterial({color:0x8f8d88});
const steelDarkMat = new THREE.MeshLambertMaterial({color:0x5e5c58});
const cardboardMat = new THREE.MeshLambertMaterial({color:0xc8a06a});
const cardDarkMat  = new THREE.MeshLambertMaterial({color:0xb08050});
const wallMat      = new THREE.MeshLambertMaterial({color:0xb0aca6});

// ── Build Warehouse ──────────────────────────────────────────
function buildWarehouse(){

  // Concrete floor with gloss
  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(26,18),
    new THREE.MeshLambertMaterial({color:0x888e96})
  );
  floor.rotation.x=-Math.PI/2;
  floor.receiveShadow=true;
  scene.add(floor);

  // Floor grid lines (subtle)
  const grid = new THREE.GridHelper(26,26,0xaaaaaa,0xaaaaaa);
  grid.position.y=0.002;
  grid.material.opacity=0.15;
  grid.material.transparent=true;
  scene.add(grid);

  // Walls
  const bwall = new THREE.Mesh(new THREE.PlaneGeometry(26,7),wallMat);
  bwall.position.set(0,3.5,-9); scene.add(bwall);
  const fwall = new THREE.Mesh(new THREE.PlaneGeometry(26,7),wallMat);
  fwall.position.set(0,3.5,9); fwall.rotation.y=Math.PI; scene.add(fwall);
  const lwall = new THREE.Mesh(new THREE.PlaneGeometry(18,7),wallMat);
  lwall.position.set(-13,3.5,0); lwall.rotation.y=Math.PI/2; scene.add(lwall);
  const rwall = new THREE.Mesh(new THREE.PlaneGeometry(18,7),wallMat);
  rwall.position.set(13,3.5,0); rwall.rotation.y=-Math.PI/2; scene.add(rwall);

  // Yellow aisle stripes
  [[0,-5],[0,5]].forEach(([x,z])=>{
    const s=new THREE.Mesh(new THREE.PlaneGeometry(26,0.12),new THREE.MeshLambertMaterial({color:0xc9a227}));
    s.rotation.x=-Math.PI/2; s.position.set(x,0.005,z); scene.add(s);
  });
  [[-6,0],[6,0]].forEach(([x,z])=>{
    const s=new THREE.Mesh(new THREE.PlaneGeometry(0.12,18),new THREE.MeshLambertMaterial({color:0xc9a227}));
    s.rotation.x=-Math.PI/2; s.position.set(x,0.005,z); scene.add(s);
  });

  // Ceiling light fixtures (up high, short)
  [-6,0,6].forEach(ax=>{
    const fix=new THREE.Mesh(
      new THREE.BoxGeometry(0.25,0.05,5),
      new THREE.MeshBasicMaterial({color:0xfffff0})
    );
    fix.position.set(ax,6.5,0); scene.add(fix);
  });

  // ── Metal Rack Shelves ────────────────────────────────────
  shelfBins = [];

  SHELF_X.forEach((sx,si)=>{
    buildRack(sx, si);
  });

  // ── Pick Zone ─────────────────────────────────────────────
  const pg=new THREE.Mesh(new THREE.PlaneGeometry(2,2),new THREE.MeshLambertMaterial({color:0x00aa44,transparent:true,opacity:0.5}));
  pg.rotation.x=-Math.PI/2; pg.position.set(PICK.x,0.01,PICK.z); scene.add(pg);
  addRing(PICK.x,PICK.z,0x00ff66);
  makeLabel('PICK',PICK.x,0.5,PICK.z,'#00cc55',0.8);

  // ── Dispatch Zone ─────────────────────────────────────────
  const dg=new THREE.Mesh(new THREE.PlaneGeometry(2,2),new THREE.MeshLambertMaterial({color:0xaa2200,transparent:true,opacity:0.5}));
  dg.rotation.x=-Math.PI/2; dg.position.set(DISPATCH.x,0.01,DISPATCH.z); scene.add(dg);
  addRing(DISPATCH.x,DISPATCH.z,0xff4444);
  makeLabel('DISPATCH',DISPATCH.x,0.5,DISPATCH.z,'#dd3333',0.8);

  // ── Home / Charging Station ───────────────────────────────
  const hm=new THREE.Mesh(new THREE.CylinderGeometry(0.55,0.55,0.04,24),new THREE.MeshLambertMaterial({color:0x2255aa,transparent:true,opacity:0.6}));
  hm.position.set(HOME.x,0.02,HOME.z); scene.add(hm);
  // Charging post
  const cp=new THREE.Mesh(new THREE.BoxGeometry(0.15,0.9,0.6),new THREE.MeshLambertMaterial({color:0x334455}));
  cp.position.set(HOME.x-0.7,0.45,HOME.z); scene.add(cp);
  makeLabel('HOME',HOME.x,0.4,HOME.z,'#4488ff',0.7);
}

// ── Build one metal rack shelf ────────────────────────────────
function buildRack(sx, si){
  const rackH   = 2.8;
  const rackD   = 0.7;
  const rackL   = 8.0;
  const levels  = [0.55, 1.2, 1.85, 2.5];
  const postR   = 0.03;

  // Vertical posts (4 corners + middle)
  const postPositions = [-3.5,0.0,3.5];
  postPositions.forEach(pz=>{
    [-rackD/2, rackD/2].forEach(pd=>{
      const post=new THREE.Mesh(
        new THREE.BoxGeometry(postR*2,rackH,postR*2),
        steelDarkMat
      );
      post.position.set(sx+pd, rackH/2, pz);
      post.castShadow=true;
      scene.add(post);
    });
    // (bracing removed for clarity)
  });

  // Shelf decks — X = shelf depth (narrow), Z = shelf length (along rack)
  levels.forEach(lh=>{
    const deck=new THREE.Mesh(
      new THREE.BoxGeometry(rackD, 0.04, rackL-1.0),
      new THREE.MeshLambertMaterial({color:0x7d7a75})
    );
    deck.position.set(sx, lh, 0);
    deck.receiveShadow=true;
    deck.castShadow=true;
    scene.add(deck);

    if(lh < 2.6){
      placeShelfItems(sx, si, lh);
    }
  });

  // Shelf label sign
  makeLabel(SHELF_NAMES[si], sx, rackH+0.35, 0, '#3399ff', 0.9);
}

// ── Place bins and boxes on a shelf level ─────────────────────
function placeShelfItems(sx, si, lh){
  // (cardboard boxes removed — clean shelves)

  // Coloured plastic bins (interactive — can be picked)
  BIN_COLORS.forEach((col,ci)=>{
    const bz = -3.0 + ci * 1.0;
    const binH=0.22, binW=0.28, binD=0.32;

    // Bin body
    const binMat=new THREE.MeshLambertMaterial({color:col});
    const bin=new THREE.Mesh(
      new THREE.BoxGeometry(binW,binH,binD),
      binMat
    );
    bin.position.set(sx+0.18, lh+binH/2, bz);
    bin.castShadow=true;
    scene.add(bin);

    // Bin rim (slightly lighter)
    const rimMat=new THREE.MeshLambertMaterial({color:new THREE.Color(col).addScalar(0.15)});
    const rim=new THREE.Mesh(
      new THREE.BoxGeometry(binW+0.02,0.03,binD+0.02),
      rimMat
    );
    rim.position.set(sx+0.18, lh+binH, bz);
    scene.add(rim);

    // Store reference for picking
    shelfBins.push({
      id:      SHELF_NAMES[si]+'_'+BIN_NAMES[ci]+'_'+lh.toFixed(1),
      mesh:    bin,
      rimMesh: rim,
      color:   BIN_NAMES[ci],
      shelfX:  sx,
      shelfIdx:si,
      z:       bz,
      level:   lh,
      visible: true
    });
  });
}

// ── Build Robot ───────────────────────────────────────────────
function buildRobot(){
  robotGroup = new THREE.Group();
  scene.add(robotGroup);

  // Mobile base — white rounded body like image
  const baseMat  = new THREE.MeshLambertMaterial({color:0xf0f0f0});
  const accentMat= new THREE.MeshLambertMaterial({color:0x222222});
  const blueMat  = new THREE.MeshLambertMaterial({color:0x2255cc});

  // Main body
  const body=new THREE.Mesh(new THREE.BoxGeometry(0.7,0.28,0.8),baseMat);
  body.position.y=0.14; body.castShadow=true; robotGroup.add(body);

  // Top dome/panel
  const dome=new THREE.Mesh(new THREE.CylinderGeometry(0.28,0.34,0.18,16),baseMat);
  dome.position.y=0.37; robotGroup.add(dome);

  // Bottom ring (dark)
  const ring=new THREE.Mesh(new THREE.CylinderGeometry(0.35,0.35,0.06,16),accentMat);
  ring.position.y=0.03; robotGroup.add(ring);

  // Wheels (4 black)
  wheelMeshes=[];
  const wg=new THREE.CylinderGeometry(0.1,0.1,0.07,16);
  const wm=new THREE.MeshLambertMaterial({color:0x111111});
  [[-0.33,-0.32],[0.33,-0.32],[-0.33,0.32],[0.33,0.32]].forEach(([wx,wz])=>{
    const w=new THREE.Mesh(wg,wm);
    w.rotation.z=Math.PI/2; w.position.set(wx,0.1,wz);
    robotGroup.add(w); wheelMeshes.push(w);
  });

  // Status light (green/red)
  const slm=new THREE.MeshBasicMaterial({color:0x00ff00});
  statusLight=new THREE.Mesh(new THREE.SphereGeometry(0.035,8,8),slm);
  statusLight.position.set(0.22,0.48,0.3);
  robotGroup.add(statusLight);

  // Orange beacon on top
  const bkm=new THREE.MeshBasicMaterial({color:0xff6600});
  const bk=new THREE.Mesh(new THREE.CylinderGeometry(0.04,0.04,0.2,10),bkm);
  bk.position.set(0,0.57,0); robotGroup.add(bk);
  const bktop=new THREE.Mesh(new THREE.SphereGeometry(0.055,8,8),bkm);
  bktop.position.set(0,0.68,0); robotGroup.add(bktop);
  beaconLight=new THREE.PointLight(0xff6600,1.5,3);
  beaconLight.position.set(0,0.7,0); robotGroup.add(beaconLight);

  // Arm base
  const am=new THREE.MeshLambertMaterial({color:0xeeeeee});
  const jm=new THREE.MeshLambertMaterial({color:0xaaaaaa});

  const armBase=new THREE.Group();
  armBase.position.set(0,0.47,0);
  robotGroup.add(armBase);

  // Arm column
  const col=new THREE.Mesh(new THREE.CylinderGeometry(0.055,0.065,0.15,12),am);
  col.position.y=0.075; armBase.add(col);

  // Joint 1
  const j1g=new THREE.Group(); j1g.position.y=0.15; armBase.add(j1g); armJ.push(j1g);
  j1g.add(new THREE.Mesh(new THREE.SphereGeometry(0.055,10,10),jm));
  const l1=new THREE.Mesh(new THREE.CylinderGeometry(0.038,0.038,0.28,10),am);
  l1.position.y=0.14; j1g.add(l1);

  // Joint 2
  const j2g=new THREE.Group(); j2g.position.y=0.28; j1g.add(j2g); armJ.push(j2g);
  j2g.add(new THREE.Mesh(new THREE.SphereGeometry(0.046,10,10),jm));
  const l2=new THREE.Mesh(new THREE.CylinderGeometry(0.033,0.033,0.24,10),am);
  l2.position.y=0.12; j2g.add(l2);

  // Joint 3
  const j3g=new THREE.Group(); j3g.position.y=0.24; j2g.add(j3g); armJ.push(j3g);
  j3g.add(new THREE.Mesh(new THREE.SphereGeometry(0.04,10,10),jm));
  const l3=new THREE.Mesh(new THREE.CylinderGeometry(0.028,0.028,0.2,10),am);
  l3.position.y=0.1; j3g.add(l3);

  // Gripper
  const j4g=new THREE.Group(); j4g.position.y=0.2; j3g.add(j4g); armJ.push(j4g);
  j4g.add(new THREE.Mesh(new THREE.BoxGeometry(0.07,0.055,0.055),am));
  const f1=new THREE.Mesh(new THREE.BoxGeometry(0.012,0.075,0.012),am);
  f1.position.set(0.022,0.1,0); j4g.add(f1);
  const f2=f1.clone(); f2.position.set(-0.022,0.1,0); j4g.add(f2);

  // Scan cone (camera FOV visualisation, hidden until scanning)
  const scanGeo=new THREE.ConeGeometry(0.6,1.4,16,1,true);
  const scanMat=new THREE.MeshBasicMaterial({color:0x00ffcc,transparent:true,opacity:0,side:THREE.DoubleSide});
  window.scanCone=new THREE.Mesh(scanGeo,scanMat);
  window.scanCone.rotation.z=Math.PI/2;  // point sideways toward shelf
  robotGroup.add(window.scanCone);

  // Held bin (hidden until picking)
  heldBin=new THREE.Mesh(
    new THREE.BoxGeometry(0.28,0.22,0.32),
    new THREE.MeshLambertMaterial({color:0xff2222,transparent:true,opacity:0})
  );
  scene.add(heldBin);
}

// ── Helpers ───────────────────────────────────────────────────
function addRing(x,z,color){
  const rg=new THREE.RingGeometry(0.65,0.82,32);
  const rm=new THREE.MeshBasicMaterial({color,side:THREE.DoubleSide,transparent:true,opacity:0.85});
  const ring=new THREE.Mesh(rg,rm);
  ring.rotation.x=-Math.PI/2; ring.position.set(x,0.04,z);
  ring.userData.ring=true; scene.add(ring);
}

function makeLabel(text,x,y,z,color,scale=1){
  const c=document.createElement('canvas');
  c.width=256; c.height=64;
  const cx=c.getContext('2d');
  cx.fillStyle=color;
  cx.font='bold 34px Arial';
  cx.textAlign='center'; cx.textBaseline='middle';
  cx.fillText(text,128,34);
  const t=new THREE.CanvasTexture(c);
  const g=new THREE.PlaneGeometry(1.6*scale,0.4*scale);
  const m=new THREE.MeshBasicMaterial({map:t,transparent:true,side:THREE.DoubleSide,depthWrite:false});
  const mesh=new THREE.Mesh(g,m);
  mesh.position.set(x,y,z);
  mesh.userData.label=true;
  scene.add(mesh);
}

// ── Animation ─────────────────────────────────────────────────
const clock=new THREE.Clock();

function animate(){
  requestAnimationFrame(animate);
  const dt=Math.min(clock.getDelta(),0.05);
  const t=clock.getElapsedTime();

  camera.position.x=sph.r*Math.sin(sph.phi)*Math.sin(sph.theta);
  camera.position.y=sph.r*Math.cos(sph.phi);
  camera.position.z=sph.r*Math.sin(sph.phi)*Math.cos(sph.theta);
  camera.lookAt(0,1.2,0);

  if(robotGroup){
    let moving=false;

    // ── Robot pose comes from the BACKEND (single source of truth) ──
    // The navigator drives the TOPP-RA trajectory with collision checking,
    // so it never enters an obstacle. Previously the browser walked the
    // waypoint list independently, drifted out of sync, and snapped back
    // on every replan - which looked like the robot repeating its motion
    // and clipping obstacles. Now we smoothly track the reported pose.
    const bp = state.robot_position || {x:robotX, y:robotZ};
    const tgx = bp.x, tgz = bp.y;
    const pdx = tgx - robotX, pdz = tgz - robotZ;
    const pd  = Math.sqrt(pdx*pdx + pdz*pdz);

    if(pd > 0.004){
      // critically-damped catch-up: smooth, no overshoot, minimal lag
      const k = Math.min(1, 12*dt);
      robotX += pdx * k;
      robotZ += pdz * k;
      if(pd > 0.012){
        robotGroup.rotation.y = Math.atan2(pdx, pdz);
        moving = true;
      }
    }
    robotGroup.position.set(robotX, 0, robotZ);

    // Wheels
    if(moving){ wheelAngle+=dt*9; wheelMeshes.forEach(w=>w.rotation.y=wheelAngle); }

    // Status light
    if(statusLight){
      const p=(Math.sin(t*4)+1)/2;
      statusLight.material.color.setRGB(moving?1:0, moving?p*0.2:p, 0);
    }

    // Beacon pulse
    if(beaconLight) beaconLight.intensity=moving?1.2+Math.sin(t*10)*0.8:0.2+Math.sin(t*1.5)*0.15;

    // Scan cone visibility
    if(window.scanCone){
      const scanning=(state.status||'').toLowerCase().includes('scan');
      const targetOp=scanning?0.25+Math.sin(t*8)*0.1:0;
      window.scanCone.material.opacity+=(targetOp-window.scanCone.material.opacity)*0.15;
      window.scanCone.position.set(0.7,0.5,0);
    }

    // Arm animation — reach toward shelf (shelf is at +X relative to robot)
    armCurrent+=(armTarget-armCurrent)*0.06;
    const reach = Math.max(0, armCurrent);
    // Negative z-rotation tilts the arm toward +X (the shelf side)
    if(armJ[0]) armJ[0].rotation.z = -reach*0.8;       // shoulder toward shelf
    if(armJ[1]) armJ[1].rotation.z = 0.3-reach*1.1;    // upper arm reach out
    if(armJ[2]) armJ[2].rotation.z = -reach*0.6;       // forearm
    if(armJ[3]) armJ[3].rotation.z = 0.4-reach*0.6;    // gripper down onto bin

    // Determine current phase from status
    const stLower=(state.status||'').toLowerCase();
    const isPicking = stLower.includes('pick');
    const isPlacing = stLower.includes('place');

    // Compute gripper world position (end of arm chain)
    let gripPos=new THREE.Vector3();
    if(armJ[3]){
      armJ[3].getWorldPosition(gripPos);
    }

    // PICK is commanded by the BACKEND. The browser removes exactly
    // the bin the perception identified — no local matching/guessing.
    if(state.remove_bin_id && state.remove_bin_id!==heldBin._lastRemoved){
      const target = shelfBins.find(b=>b.id===state.remove_bin_id && b.visible);
      if(target){
        heldBin._lastRemoved = state.remove_bin_id;
        target.visible=false;
        target.mesh.visible=false;
        target.rimMesh.visible=false;
        heldBin.material.color.copy(target.mesh.material.color);
        heldBin.material.opacity=1;
        console.log('BACKEND COMMANDED REMOVAL:', target.id);
        publishWorldState();   // world changed — tell the backend
      }
    }

    // Reset pick flag & release bin when not holding (placed)
    if(!state.holding){
      heldBin._wasPicked=false;
      heldBin._grabbed=null;
      heldBin.material.opacity+=(0-heldBin.material.opacity)*0.25;
      if(heldBin.material.opacity<0.02) heldBin.material.opacity=0;
    }
  }

  // Pulse rings + billboard labels
  scene.children.forEach(c=>{
    if(c.userData.ring){ c.rotation.z+=dt*0.4; c.material.opacity=0.55+(Math.sin(t*2)+1)*0.2; }
    if(c.userData.label) c.lookAt(camera.position);
  });

  renderer.render(scene,camera);
}

// ── RRT Path ──────────────────────────────────────────────────
function applyRRTPath(pts){
  // Visualisation only. The robot's motion is driven by the backend pose,
  // not by walking this list - see the pose tracking in animate().
  if(rrtLine) scene.remove(rrtLine);
  if(pts.length>1){
    const points=pts.map(p=>new THREE.Vector3(p.x,0.12,p.y));
    const geo=new THREE.BufferGeometry().setFromPoints(points);
    const mat=new THREE.LineBasicMaterial({color:0x00ffcc,linewidth:2});
    rrtLine=new THREE.Line(geo,mat);
    scene.add(rrtLine);
  }
  rrtPath=pts;
}

// ── Dynamic Obstacles ─────────────────────────────────────────
function updateDynObs(obstacles){
  dynObsMeshes.forEach(m=>scene.remove(m));
  dynObsMeshes=[];
  if(!obstacles||!obstacles.length) return;
  obstacles.forEach(obs=>{
    // Orange warning barrier
    const g=new THREE.BoxGeometry(0.55,1.1,0.55);
    const m=new THREE.MeshLambertMaterial({color:0xff6600});
    const mesh=new THREE.Mesh(g,m);
    mesh.position.set(obs.cx,0.55,obs.cy);
    mesh.castShadow=true;
    scene.add(mesh);
    dynObsMeshes.push(mesh);

    // Warning stripes
    const stripe=new THREE.Mesh(
      new THREE.BoxGeometry(0.57,0.08,0.57),
      new THREE.MeshBasicMaterial({color:0x111111})
    );
    stripe.position.set(obs.cx,0.9,obs.cy);
    scene.add(stripe);
    dynObsMeshes.push(stripe);

    // Flashing light
    const pl=new THREE.PointLight(0xff4400,2.0,4);
    pl.position.set(obs.cx,1.5,obs.cy);
    scene.add(pl);
    dynObsMeshes.push(pl);
  });
}

// ── Poll ──────────────────────────────────────────────────────
async function poll(){
  try{
    const r=await fetch('/state');
    state=await r.json();

    document.getElementById('stxt').textContent=state.status||'Ready';
    document.getElementById('px').textContent=((state.robot_position?.x)||0).toFixed(2)+'m';
    document.getElementById('py').textContent=((state.robot_position?.y)||0).toFixed(2)+'m';
    document.getElementById('ox').textContent=((state.robot_position?.x)||0).toFixed(2);
    document.getElementById('oy').textContent=((state.robot_position?.y)||0).toFixed(2);
    document.getElementById('hld').textContent=state.holding?(state.target_color||'Object'):'Nothing';

    const st=state.current_step||0, tot=state.total_steps||0;
    document.getElementById('stp').textContent=tot>0?st+'/'+tot:'-';
    document.getElementById('ostp').textContent=tot>0?st+'/'+tot:'-';
    document.getElementById('prg').style.width=tot>0?(st/tot*100)+'%':'0%';

    document.getElementById('lb').innerHTML=(state.task_log||[]).slice().reverse().slice(0,15)
      .map(e=>`<div class="le i">${e}</div>`).join('');

    const s2=state.status||'';
    if(s2.includes('Pick')||s2.includes('pick')) armTarget=1.0;      // reach out to shelf
    else if(s2.includes('Place')||s2.includes('place')) armTarget=-0.6; // extend to dispatch
    else if(s2.includes('Scan')||s2.includes('scan')) armTarget=0.3;  // slight raise (camera up)
    else armTarget=0;

    if(state.planned_path&&state.planned_path.length>1){
      // Key on the ENTIRE path, not just the goal. On a replan the goal is
      // unchanged but the route differs — keying on the endpoint made the
      // browser ignore reroutes and keep driving through the new obstacle.
      const key=JSON.stringify(state.planned_path);
      if(key!==poll._lastKey){
        poll._lastKey=key;
        applyRRTPath(state.planned_path);
      }
    }

    updateDynObs(state.dynamic_obstacles);
  }catch(e){}
  setTimeout(poll,110);
}
poll._lastKey='';

// ── Controls ──────────────────────────────────────────────────
function q(t){ document.getElementById('oi').value=t; }
function sendQ(order){ document.getElementById('oi').value=order; sendOrder(); }

async function sendOrder(){
  const o=document.getElementById('oi').value.trim(); if(!o)return;
  addLog('> '+o,'i');
  await fetch('/order',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({order:o})});
  document.getElementById('oi').value='';
}

async function eStop(){
  addLog('E-STOP','e');
  rrtPath=[];
  await fetch('/order',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({order:'EMERGENCY_STOP'})});
  document.getElementById('badge').className='badge offline';
  document.getElementById('badge').textContent='E-STOP';
}

function addLog(msg,type){
  const lb=document.getElementById('lb');
  const d=document.createElement('div');
  d.className='le '+type; d.textContent=msg;
  lb.insertBefore(d,lb.firstChild);
  if(lb.children.length>20) lb.removeChild(lb.lastChild);
}

// ── Publish the REAL world state (which bins exist) ──────────
async function publishWorldState(){
  try{
    const bins = shelfBins
      .filter(b=>b.visible)
      .map(b=>({ id:b.id, color:b.color,
                 x:b.shelfX, z:b.mesh.position.z, visible:true }));
    await fetch('/world_state',{
      method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify({bins:bins})
    });
  }catch(e){ console.log('world state err',e); }
}

// ── Robot camera capture for REAL vision ─────────────────────
function captureRobotCamera(){
  // Position the robot camera at the robot, looking at the shelf (+X side)
  robotCam.position.set(robotX+0.3, 0.8, robotZ);
  robotCam.lookAt(robotX+2.0, 0.8, robotZ);   // look toward shelf

  // Render robot's view into a small offscreen canvas
  const camW=320, camH=200;
  const rt = captureRobotCamera._rt || (captureRobotCamera._rt =
    new THREE.WebGLRenderTarget(camW, camH));

  renderer.setRenderTarget(rt);
  renderer.render(scene, robotCam);
  renderer.setRenderTarget(null);

  // Read pixels
  const pixels = new Uint8Array(camW*camH*4);
  renderer.readRenderTargetPixels(rt, 0, 0, camW, camH, pixels);

  // Draw to a 2D canvas to get a PNG (flip Y — WebGL is bottom-up)
  const cnv = captureRobotCamera._cnv || (captureRobotCamera._cnv =
    document.createElement('canvas'));
  cnv.width=camW; cnv.height=camH;
  const ctx=cnv.getContext('2d');
  const imgData=ctx.createImageData(camW,camH);
  for(let y=0;y<camH;y++){
    for(let x=0;x<camW;x++){
      const src=((camH-1-y)*camW+x)*4;   // flip Y
      const dst=(y*camW+x)*4;
      imgData.data[dst]=pixels[src];
      imgData.data[dst+1]=pixels[src+1];
      imgData.data[dst+2]=pixels[src+2];
      imgData.data[dst+3]=255;
    }
  }
  ctx.putImageData(imgData,0,0);
  return cnv.toDataURL('image/png');
}

async function sendRobotCameraFrame(){
  try{
    const img=captureRobotCamera();
    // The camera looks along Z, so the shelf bins span the image width in Z.
    // Robot Z is centre; the visible Z span is roughly robotZ-4 to robotZ+4,
    // but bins are at fixed shelf Z range -3..+3 regardless of robot position.
    await fetch('/camera_image',{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({ image: img, shelf_span: [-3.0, 3.0] })
    });
  }catch(e){ console.log('cam send err',e); }
}


// ═══════════════════════════════════════════════════════════
//  PLANNER SCHEMATIC  —  2D SCADA-style engineering view
//  Shows what the 3D view cannot: the RRT* search tree,
//  obstacle inflation (Minkowski buffer), and the TOPP-RA
//  velocity profile.
// ═══════════════════════════════════════════════════════════
const scCanvas = document.getElementById('sc');
const scx = scCanvas.getContext('2d');

// World bounds -> canvas mapping
const W_MIN_X=-8, W_MAX_X=8, W_MIN_Y=-6.5, W_MAX_Y=6.5;
const SC_TOP=26, SC_BOT=58;      // header / status bar heights

let velHistory = [];             // {t, v} for the profile strip
let lastSchemPos = null, lastSchemT = 0;

function scResize(){
  const r = scCanvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio,2);
  scCanvas.width  = r.width*dpr;
  scCanvas.height = r.height*dpr;
  scx.setTransform(dpr,0,0,dpr,0,0);
}
window.addEventListener('resize', scResize);

function scGeom(){
  const r = scCanvas.getBoundingClientRect();
  const padX = 26;
  const top = SC_TOP+14, bot = r.height-SC_BOT-72;   // leave room for vel strip
  const availW = r.width-padX*2, availH = bot-top;
  const sx = availW/(W_MAX_X-W_MIN_X);
  const sy = availH/(W_MAX_Y-W_MIN_Y);
  const s  = Math.min(sx,sy);
  const ox = padX + (availW-(W_MAX_X-W_MIN_X)*s)/2;
  const oy = top  + (availH-(W_MAX_Y-W_MIN_Y)*s)/2;
  return {s,ox,oy,r,bot};
}
const wx2c = (x,g)=> g.ox + (x-W_MIN_X)*g.s;
const wy2c = (y,g)=> g.oy + (y-W_MIN_Y)*g.s;

function drawSchematic(){
  const r = scCanvas.getBoundingClientRect();
  if(r.width<10){ requestAnimationFrame(drawSchematic); return; }
  if(scCanvas.width !== Math.round(r.width*Math.min(window.devicePixelRatio,2))) scResize();

  const g = scGeom();
  const tree = state.rrt_tree || {edges:[],obstacles:[],radius:0.35,nodes:0};

  // ── background ──
  scx.fillStyle='#0b0e12';
  scx.fillRect(0,0,r.width,r.height);

  // ── grid ──
  scx.strokeStyle='#151b22'; scx.lineWidth=1;
  scx.beginPath();
  for(let x=W_MIN_X;x<=W_MAX_X;x+=1){
    const cx=wx2c(x,g); scx.moveTo(cx,wy2c(W_MIN_Y,g)); scx.lineTo(cx,wy2c(W_MAX_Y,g));
  }
  for(let y=W_MIN_Y;y<=W_MAX_Y;y+=1){
    const cy=wy2c(y,g); scx.moveTo(wx2c(W_MIN_X,g),cy); scx.lineTo(wx2c(W_MAX_X,g),cy);
  }
  scx.stroke();

  // major axes
  scx.strokeStyle='#1e2731'; scx.lineWidth=1.2;
  scx.beginPath();
  scx.moveTo(wx2c(0,g),wy2c(W_MIN_Y,g)); scx.lineTo(wx2c(0,g),wy2c(W_MAX_Y,g));
  scx.moveTo(wx2c(W_MIN_X,g),wy2c(0,g)); scx.lineTo(wx2c(W_MAX_X,g),wy2c(0,g));
  scx.stroke();

  // ── obstacle inflation (Minkowski buffer) ──
  const rad = tree.radius || 0.35;
  (tree.obstacles||[]).forEach(o=>{
    const x0=wx2c(o.cx-o.w/2-rad,g), y0=wy2c(o.cy-o.h/2-rad,g);
    const w =(o.w+2*rad)*g.s,        h =(o.h+2*rad)*g.s;
    scx.setLineDash([3,3]);
    scx.strokeStyle='#3a2f22'; scx.lineWidth=1;
    scx.strokeRect(x0,y0,w,h);
    scx.setLineDash([]);
  });

  // ── obstacles (hatched) ──
  (tree.obstacles||[]).forEach(o=>{
    const x0=wx2c(o.cx-o.w/2,g), y0=wy2c(o.cy-o.h/2,g);
    const w=o.w*g.s, h=o.h*g.s;
    scx.fillStyle='#161d25'; scx.fillRect(x0,y0,w,h);
    scx.save(); scx.beginPath(); scx.rect(x0,y0,w,h); scx.clip();
    scx.strokeStyle='#25303c'; scx.lineWidth=1;
    scx.beginPath();
    for(let i=-h;i<w+h;i+=7){ scx.moveTo(x0+i,y0+h); scx.lineTo(x0+i+h,y0); }
    scx.stroke(); scx.restore();
    scx.strokeStyle='#33414f'; scx.lineWidth=1.2; scx.strokeRect(x0,y0,w,h);
  });

  // ── RRT* search tree ──
  const edges = tree.edges||[];
  if(edges.length){
    scx.strokeStyle='rgba(74,140,190,0.30)'; scx.lineWidth=0.7;
    scx.beginPath();
    edges.forEach(e=>{
      scx.moveTo(wx2c(e[0],g),wy2c(e[1],g));
      scx.lineTo(wx2c(e[2],g),wy2c(e[3],g));
    });
    scx.stroke();
  }

  // ── zones ──
  const zone=(x,y,col,label)=>{
    scx.strokeStyle=col; scx.lineWidth=1.2;
    scx.strokeRect(wx2c(x-1,g),wy2c(y-1,g),2*g.s,2*g.s);
    scx.fillStyle=col; scx.font='9px monospace';
    scx.fillText(label, wx2c(x-1,g), wy2c(y-1,g)-4);
  };
  zone(-6,5,'#3f8f5f','PICK');
  zone( 6,5,'#a8534a','DISPATCH');
  zone(-6,-5,'#4a6f9c','HOME');

  // ── planned path ──
  const path = state.planned_path||[];
  if(path.length>1){
    scx.strokeStyle='#4fd6b8'; scx.lineWidth=1.8;
    scx.beginPath();
    path.forEach((p,i)=>{
      const cx=wx2c(p.x,g), cy=wy2c(p.y,g);
      i?scx.lineTo(cx,cy):scx.moveTo(cx,cy);
    });
    scx.stroke();
    scx.fillStyle='#4fd6b8';
    path.forEach(p=>{
      scx.beginPath(); scx.arc(wx2c(p.x,g),wy2c(p.y,g),2.2,0,6.28); scx.fill();
    });
  }

  // ── dynamic obstacles ──
  (state.dynamic_obstacles||[]).forEach(o=>{
    const cx=wx2c(o.cx,g), cy=wy2c(o.cy,g);
    scx.strokeStyle='#d98324'; scx.lineWidth=1.6;
    scx.strokeRect(cx-0.4*g.s,cy-0.4*g.s,0.8*g.s,0.8*g.s);
    scx.beginPath(); scx.arc(cx,cy,(0.4+rad)*g.s,0,6.28);
    scx.setLineDash([2,3]); scx.strokeStyle='#6b4a1e'; scx.lineWidth=1;
    scx.stroke(); scx.setLineDash([]);
  });

  // ── robot ──
  const rp = state.robot_position||{x:0,y:0};
  const rcx=wx2c(rp.x,g), rcy=wy2c(rp.y,g);

  // inflation circle
  scx.beginPath(); scx.arc(rcx,rcy,rad*g.s,0,6.28);
  scx.strokeStyle='rgba(143,211,199,0.35)'; scx.lineWidth=1; scx.stroke();

  // heading from last motion
  let hd=0;
  if(lastSchemPos){
    const dx=rp.x-lastSchemPos.x, dy=rp.y-lastSchemPos.y;
    if(Math.hypot(dx,dy)>0.002) hd=Math.atan2(dy,dx);
    else hd=lastSchemPos.h||0;
  }
  scx.save(); scx.translate(rcx,rcy); scx.rotate(hd);
  scx.fillStyle='#8fd3c7';
  scx.beginPath(); scx.moveTo(9,0); scx.lineTo(-6,5.5); scx.lineTo(-6,-5.5); scx.closePath(); scx.fill();
  scx.restore();

  // crosshair
  scx.strokeStyle='rgba(143,211,199,0.5)'; scx.lineWidth=0.8;
  scx.beginPath();
  scx.moveTo(rcx-11,rcy); scx.lineTo(rcx-5,rcy);
  scx.moveTo(rcx+5,rcy);  scx.lineTo(rcx+11,rcy);
  scx.moveTo(rcx,rcy-11); scx.lineTo(rcx,rcy-5);
  scx.moveTo(rcx,rcy+5);  scx.lineTo(rcx,rcy+11);
  scx.stroke();

  // ── LIDAR: field of view, rays, and returns ──
  const ld = state.lidar||{hits:[],ranges:[],fov:270,max_range:3.5};
  if(ld.ranges && ld.ranges.length){
    const fov = (ld.fov||270)*Math.PI/180;
    const a0  = hd - fov/2;
    const step= fov/Math.max(1,ld.ranges.length-1);

    // swept area
    scx.save(); scx.translate(rcx,rcy);
    scx.fillStyle='rgba(79,214,184,0.05)';
    scx.beginPath(); scx.moveTo(0,0);
    scx.arc(0,0,(ld.max_range||3.5)*g.s, a0-hd, a0-hd+fov);
    scx.closePath(); scx.fill();
    scx.restore();

    // individual rays
    scx.strokeStyle='rgba(79,214,184,0.16)'; scx.lineWidth=0.5;
    scx.beginPath();
    ld.ranges.forEach((r,i)=>{
      const a=a0+i*step;
      scx.moveTo(rcx,rcy);
      scx.lineTo(rcx+Math.cos(a)*r*g.s, rcy+Math.sin(a)*r*g.s);
    });
    scx.stroke();

    // FOV boundary
    scx.strokeStyle='rgba(79,214,184,0.4)'; scx.lineWidth=1;
    scx.beginPath();
    scx.moveTo(rcx,rcy);
    scx.lineTo(rcx+Math.cos(a0)*(ld.max_range||3.5)*g.s,
               rcy+Math.sin(a0)*(ld.max_range||3.5)*g.s);
    scx.moveTo(rcx,rcy);
    const a1=a0+fov;
    scx.lineTo(rcx+Math.cos(a1)*(ld.max_range||3.5)*g.s,
               rcy+Math.sin(a1)*(ld.max_range||3.5)*g.s);
    scx.stroke();

    // returns (points where rays actually hit something)
    scx.fillStyle='#ff5544';
    (ld.hits||[]).forEach(h=>{
      scx.beginPath();
      scx.arc(wx2c(h[0],g), wy2c(h[1],g), 1.9, 0, 6.28);
      scx.fill();
    });
  }

  // ── velocity profile strip (TOPP-RA) ──
  const vy0=g.bot+16, vh=52, vx0=26, vw=r.width-52;
  scx.strokeStyle='#1e2731'; scx.lineWidth=1;
  scx.strokeRect(vx0,vy0,vw,vh);
  scx.fillStyle='#4d5966'; scx.font='8px monospace';
  scx.fillText('VELOCITY  m/s', vx0+4, vy0-4);

  // update history
  const now=performance.now();
  if(lastSchemPos && now-lastSchemT>0){
    const d=Math.hypot(rp.x-lastSchemPos.x, rp.y-lastSchemPos.y);
    const v=d/((now-lastSchemT)/1000);
    velHistory.push(Math.min(v,2.5));
    if(velHistory.length>240) velHistory.shift();
  }
  lastSchemPos={x:rp.x,y:rp.y,h:hd}; lastSchemT=now;

  if(velHistory.length>1){
    scx.strokeStyle='#8fd3c7'; scx.lineWidth=1.3;
    scx.beginPath();
    velHistory.forEach((v,i)=>{
      const px=vx0+(i/239)*vw;
      const py=vy0+vh-(v/2.5)*vh;
      i?scx.lineTo(px,py):scx.moveTo(px,py);
    });
    scx.stroke();
  }

  // ── status cells ──
  document.getElementById('scNodes').textContent = tree.nodes||0;
  document.getElementById('scPts').textContent   = (ld.hits||[]).length;
  document.getElementById('scInf').textContent   = rad.toFixed(2)+' m';
  const st=(state.status||'').toLowerCase();
  document.getElementById('scState').textContent =
    (state.status||'').includes('SENSED')?'DETECTED':
    st.includes('scan')?'SCANNING':
    st.includes('pick')?'GRASP':
    st.includes('place')?'RELEASE':
    st.includes('complete')?'DONE':
    path.length>1?'TRACKING':'IDLE';

  requestAnimationFrame(drawSchematic);
}

window.onload=()=>{
  resize();
  buildWarehouse();
  buildRobot();
  robotX=HOME.x; robotZ=HOME.z;
  robotGroup.position.set(robotX,0,robotZ);
  animate();
  scResize();
  drawSchematic();
  poll();
  publishWorldState();
  setInterval(publishWorldState, 3000);
};
</script>
</body>
</html>'''


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self,format,*args): pass

    def do_GET(self):
        if self.path=='/':
            self.send_response(200)
            self.send_header('Content-Type','text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path=='/state':
            self.send_response(200)
            self.send_header('Content-Type','application/json')
            self.send_header('Access-Control-Allow-Origin','*')
            self.end_headers()
            self.wfile.write(json.dumps(STATE).encode())
        else:
            self.send_error(404)

    def do_POST(self):
        length=int(self.headers.get('Content-Length',0))
        body=self.rfile.read(length)
        try: data=json.loads(body)
        except: self.send_error(400); return
        if self.path=='/order':
            order=data.get('order','')
            if order and hasattr(self,'ros_node'):
                msg=String(); msg.data=order
                self.ros_node.order_pub.publish(msg)
                STATE['task_log'].append(f'> {order}')
                STATE['task_log']=STATE['task_log'][-20:]
        elif self.path=='/world_state':
            # Browser publishes the bins that actually exist right now
            if hasattr(self,'ros_node'):
                msg=String(); msg.data=json.dumps(data.get('bins',[]))
                self.ros_node.world_pub.publish(msg)
                STATE['world_bins']=data.get('bins',[])
        elif self.path=='/camera_image':
            # Browser sent a rendered camera frame — forward to image_mapper
            if hasattr(self,'ros_node'):
                msg=String()
                msg.data=json.dumps({
                    "image": data.get('image',''),
                    "shelf_span": data.get('shelf_span',[-3.0,3.0])
                })
                self.ros_node.camera_pub.publish(msg)
        self.send_response(200)
        self.send_header('Content-Type','application/json')
        self.end_headers()
        self.wfile.write(json.dumps({'status':'ok'}).encode())


class DashboardServerNode(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        self.order_pub=self.create_publisher(String,'/warehouse/order',10)
        self.camera_pub=self.create_publisher(String,'/warehouse/camera_image',10)
        self.world_pub=self.create_publisher(String,'/warehouse/world_state',10)
        self.create_subscription(String,'/warehouse/status',         self.status_cb,  10)
        self.create_subscription(String,'/warehouse/robot_position', self.position_cb,10)
        self.create_subscription(String,'/warehouse/waypoints',      self.wp_cb,      10)
        self.create_subscription(String,'/warehouse/planned_path',   self.path_cb,    10)
        self.create_subscription(String,'/warehouse/dynamic_obstacles',self.dynobs_cb,10)
        self.create_subscription(String,'/warehouse/bin_picked',self.binpicked_cb,10)
        self.create_subscription(String,'/warehouse/rrt_tree',self.tree_cb,10)
        self.create_subscription(String,'/warehouse/lidar_scan',self.scan_cb,10)
        node=self
        class H(DashboardHandler): ros_node=node
        server=HTTPServer(('0.0.0.0',8080),H)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        self.get_logger().info('Dashboard at http://localhost:8080')

    def status_cb(self,msg):
        STATE['status']=msg.data
        STATE['task_log'].append(msg.data)
        STATE['task_log']=STATE['task_log'][-20:]
        import re
        m=re.match(r'Step (\d+)/(\d+)',msg.data)
        if m:
            STATE['current_step']=int(m.group(1))
            STATE['total_steps']=int(m.group(2))
        if 'Pick' in msg.data or 'pick' in msg.data:
            STATE['holding']=True
            for color in ['red','blue','green','yellow','purple','orange','cyan']:
                if color in msg.data.lower():
                    STATE['target_color']=color; break
        if 'Place' in msg.data or 'complete' in msg.data.lower():
            STATE['holding']=False
            STATE['target_color']=None

    def position_cb(self,msg):
        STATE['robot_position']=json.loads(msg.data)

    def wp_cb(self,msg):
        wps=json.loads(msg.data)
        STATE['waypoints']=wps
        STATE['total_steps']=len(wps)

    def path_cb(self,msg):
        STATE['planned_path']=json.loads(msg.data)

    def dynobs_cb(self,msg):
        STATE['dynamic_obstacles']=json.loads(msg.data)

    def scan_cb(self,msg):
        try: STATE['lidar']=json.loads(msg.data)
        except Exception: pass

    def tree_cb(self,msg):
        try: STATE['rrt_tree']=json.loads(msg.data)
        except Exception: pass

    def binpicked_cb(self,msg):
        # Backend commands removal of ONE specific bin by id
        try:
            d=json.loads(msg.data)
            STATE['remove_bin_id']=d.get('bin_id') or None
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node=DashboardServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()