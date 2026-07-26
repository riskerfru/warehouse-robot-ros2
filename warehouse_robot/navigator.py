# =============================================================
#   NAVIGATOR NODE - RRT* + TOPP-RA + Autonomous Scan-Pick
#   Robot navigates to shelf, requests a scan, waits for the
#   detected bin position, then navigates to the EXACT bin.
#   No hardcoded bin positions — robot discovers them itself.
# =============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import random


class PerlinNoise:
    def __init__(self, seed=42):
        random.seed(seed)
        self.perm = list(range(256))
        random.shuffle(self.perm)
        self.perm += self.perm
    def fade(self,t): return t*t*t*(t*(t*6-15)+10)
    def lerp(self,t,a,b): return a+t*(b-a)
    def grad(self,h,x):
        h=h&15; g=1+(h&7)
        if h&8: g=-g
        return g*x
    def noise(self,x):
        X=int(math.floor(x))&255
        x-=math.floor(x)
        u=self.fade(x)
        a=self.perm[X]; b=self.perm[X+1]
        return self.lerp(u,self.grad(a,x),self.grad(b,x-1))


class WarehouseRRT:
    def __init__(self):
        self.x_min,self.x_max=-8.0,8.0
        self.y_min,self.y_max=-6.0,6.0
        self.obstacles=[]
        self.tree_edges=[]
        self.max_iter=5000
        self.step_size=0.25
        self.goal_bias=0.1
        self.ROBOT_RADIUS=0.35

    def set_obstacles(self,o): self.obstacles=o

    def plan(self,start,goal):
        self.tree_edges=[]            # search tree edges for visualisation
        sx,sy=float(start[0]),float(start[1])
        gx,gy=float(goal[0]),float(goal[1])
        if self._path_clear((sx,sy),(gx,gy)):
            return [(sx,sy),(gx,gy)]
        tree=[{"pos":(sx,sy),"parent":None,"cost":0.0}]
        best=None; best_cost=float('inf')
        for _ in range(self.max_iter):
            if random.random()<self.goal_bias: rx,ry=gx,gy
            else:
                rx=random.uniform(self.x_min,self.x_max)
                ry=random.uniform(self.y_min,self.y_max)
            near=min(tree,key=lambda n:math.hypot(n["pos"][0]-rx,n["pos"][1]-ry))
            nx,ny=near["pos"]
            dx,dy=rx-nx,ry-ny; d=math.hypot(dx,dy)
            if d<0.001: continue
            if d>self.step_size: rx=nx+dx/d*self.step_size; ry=ny+dy/d*self.step_size
            if not self._collision_free((rx,ry)): continue
            if not self._path_clear((nx,ny),(rx,ry)): continue
            new={"pos":(rx,ry),"parent":near,"cost":near["cost"]+math.hypot(rx-nx,ry-ny)}
            self.tree_edges.append((nx,ny,rx,ry))
            for node in tree:
                dd=math.hypot(node["pos"][0]-rx,node["pos"][1]-ry)
                if dd<1.8 and self._path_clear(node["pos"],(rx,ry)):
                    nc=node["cost"]+dd
                    if nc<new["cost"]: new["parent"]=node; new["cost"]=nc
            tree.append(new)
            dg=math.hypot(rx-gx,ry-gy)
            if dg<self.step_size*1.5:
                tc=new["cost"]+dg
                if tc<best_cost: best_cost=tc; best=new
        if best is None:
            best=min(tree,key=lambda n:math.hypot(n["pos"][0]-gx,n["pos"][1]-gy))
        path=[]; node=best
        while node: path.append(node["pos"]); node=node["parent"]
        path.reverse(); path.append((gx,gy))
        return self._smooth(path)

    def _collision_free(self,pt):
        x,y=pt
        if x<self.x_min or x>self.x_max or y<self.y_min or y>self.y_max: return False
        for o in self.obstacles:
            if abs(x-o["cx"])<o["w"]/2+self.ROBOT_RADIUS and abs(y-o["cy"])<o["h"]/2+self.ROBOT_RADIUS:
                return False
        return True

    def _path_clear(self,p1,p2,steps=20):
        x1,y1=p1; x2,y2=p2
        for i in range(steps+1):
            t=i/steps
            if not self._collision_free((x1+t*(x2-x1),y1+t*(y2-y1))): return False
        return True

    def _smooth(self,path):
        if len(path)<3: return path
        out=[path[0]]; i=0
        while i<len(path)-1:
            j=len(path)-1
            while j>i+1:
                if self._path_clear(out[-1],path[j]): break
                j-=1
            out.append(path[j]); i=j
        return out


class TOPPRAProfile:
    def __init__(self,max_vel=1.5,max_acc=0.8):
        self.max_vel=max_vel; self.max_acc=max_acc
    def generate(self,path):
        if len(path)<2: return [(path[0][0],path[0][1],0.0)]
        timed=[(path[0][0],path[0][1],0.0)]; t=0.0
        for i in range(1,len(path)):
            x1,y1=path[i-1]; x2,y2=path[i]
            dist=math.hypot(x2-x1,y2-y1)
            if dist<0.001: continue
            t_acc=self.max_vel/self.max_acc
            d_acc=0.5*self.max_acc*t_acc**2
            if dist<2*d_acc: t_seg=2*math.sqrt(dist/self.max_acc)
            else: t_seg=(dist-2*d_acc)/self.max_vel+2*t_acc
            n=max(int(t_seg/0.08),3)
            for k in range(1,n+1):
                f=k/n
                timed.append((x1+f*(x2-x1),y1+f*(y2-y1),t+f*t_seg))
            t+=t_seg
        return timed




class Lidar2D:
    """
    Simulated 2D scanning lidar.

    Casts rays from the robot pose across a limited field of view out to a
    maximum range. Returns the first intersection with any obstacle in the
    WORLD. The robot only learns about an obstacle when a ray actually hits
    it — nothing is known in advance.

    Modelled on a Sick/Hokuyo class planar scanner.
    """

    def __init__(self, fov_deg=270.0, max_range=3.5,
                 n_rays=90, range_noise=0.02):
        self.fov       = math.radians(fov_deg)
        self.max_range = max_range
        self.n_rays    = n_rays
        self.range_noise = range_noise

    def scan(self, rx, ry, heading, world_obstacles):
        """
        Returns (ranges, hit_points).
        ranges     : list of measured distance per ray (max_range if no hit)
        hit_points : list of (x,y) world coords for rays that hit something
        """
        ranges = []
        hits   = []
        start  = heading - self.fov/2.0
        step   = self.fov / max(1, self.n_rays-1)

        for i in range(self.n_rays):
            a  = start + i*step
            dx, dy = math.cos(a), math.sin(a)
            r  = self._cast(rx, ry, dx, dy, world_obstacles)
            if r < self.max_range:
                r = max(0.0, r + random.gauss(0, self.range_noise))
                hits.append((rx + dx*r, ry + dy*r))
            ranges.append(min(r, self.max_range))
        return ranges, hits

    def _cast(self, ox, oy, dx, dy, obstacles, step=0.05):
        """March along the ray until it enters an obstacle AABB."""
        t = 0.0
        while t < self.max_range:
            t += step
            px, py = ox + dx*t, oy + dy*t
            for o in obstacles:
                if (abs(px-o["cx"]) < o["w"]/2 and
                    abs(py-o["cy"]) < o["h"]/2):
                    return t
        return self.max_range


class Costmap:
    """
    Layered costmap, following the Nav2 model.

      static layer   — the known map (shelf racking)
      obstacle layer — obstacles the lidar has actually sensed
      inflation      — robot radius applied at query time

    The planner only ever sees static + sensed. Ground truth is held
    separately by the world and is never consulted for planning.
    """

    def __init__(self, robot_radius=0.35):
        self.static_layer   = []
        self.obstacle_layer = []
        self.robot_radius   = robot_radius
        self._cell = 0.35          # clustering resolution for hit points

    def set_static(self, obs):
        self.static_layer = list(obs)

    def integrate_scan(self, hit_points):
        """
        Fold lidar returns into the obstacle layer. Points are clustered
        onto a grid so repeated hits on the same object don't accumulate.
        Returns True if anything new was added.
        """
        added = False
        for (hx, hy) in hit_points:
            gx = round(hx/self._cell)*self._cell
            gy = round(hy/self._cell)*self._cell
            # ignore returns that are explained by the known static map
            if self._in_static(gx, gy):
                continue
            if not any(abs(gx-c["cx"]) < self._cell*0.6 and
                       abs(gy-c["cy"]) < self._cell*0.6
                       for c in self.obstacle_layer):
                self.obstacle_layer.append(
                    {"cx":gx,"cy":gy,"w":self._cell,"h":self._cell})
                added = True
        return added

    def _in_static(self, x, y, margin=0.25):
        for o in self.static_layer:
            if (abs(x-o["cx"]) < o["w"]/2 + margin and
                abs(y-o["cy"]) < o["h"]/2 + margin):
                return True
        return False

    def clear_obstacle_layer(self):
        """Recovery behaviour: forget sensed obstacles and re-sense."""
        n = len(self.obstacle_layer)
        self.obstacle_layer = []
        return n

    def as_obstacle_list(self):
        return self.static_layer + self.obstacle_layer


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator')

        self.create_subscription(String,'/warehouse/waypoints',self.waypoints_callback,10)
        self.create_subscription(String,'/warehouse/scan_result',self.scan_result_callback,10)

        self.status_pub   = self.create_publisher(String,'/warehouse/status',10)
        self.position_pub = self.create_publisher(String,'/warehouse/robot_position',10)
        self.path_pub     = self.create_publisher(String,'/warehouse/planned_path',10)
        self.arm_pub      = self.create_publisher(String,'/warehouse/arm_command',10)
        self.obstacle_pub = self.create_publisher(String,'/warehouse/dynamic_obstacles',10)
        self.scan_req_pub = self.create_publisher(String,'/warehouse/scan_request',10)
        self.bin_picked_pub = self.create_publisher(String,'/warehouse/bin_picked',10)
        self.tree_pub       = self.create_publisher(String,'/warehouse/rrt_tree',10)
        self.scan_pub       = self.create_publisher(String,'/warehouse/lidar_scan',10)

        self.rrt=WarehouseRRT()
        self.topp=TOPPRAProfile()

        # Perception + world model
        self.lidar   = Lidar2D(fov_deg=270.0, max_range=3.5, n_rays=90)
        self.costmap = Costmap(robot_radius=self.rrt.ROBOT_RADIUS)

        # GROUND TRUTH: obstacles that physically exist in the world.
        # The robot cannot read this — it only learns via the lidar.
        self.world_obstacles = []

        self.heading = 0.0
        self._last_scan_pub = 0.0

        self.noise_x=PerlinNoise(42); self.noise_y=PerlinNoise(137)
        self.noise_t=0.0; self.noise_speed=0.5; self.noise_amp=0.08

        self.robot_x=-6.0; self.robot_y=-4.0
        self.is_executing=False
        self._last_scan_color=None
        self._last_scan_shelf=0.0
        self._detected_bin_id=''
        self.dynamic_obstacles=[]

        # Scan result storage
        self.scan_result=None
        self.scan_received=False

        self._setup_obstacles()
        self.create_timer(0.05,self.publish_position)

        self.get_logger().info('Navigator Node started')
        self.get_logger().info('RRT* available: True')
        self.get_logger().info('Mode: Autonomous scan-then-pick (no hardcoded bin positions)')

    def _setup_obstacles(self):
        self.costmap.set_static([
            {"cx":-3.0,"cy":0.0,"w":1.0,"h":9.0},
            {"cx": 0.0,"cy":0.0,"w":1.0,"h":9.0},
            {"cx": 3.0,"cy":0.0,"w":1.0,"h":9.0},
        ])
        self.rrt.set_obstacles([
            {"cx":-3.0,"cy":0.0,"w":1.0,"h":9.0},
            {"cx": 0.0,"cy":0.0,"w":1.0,"h":9.0},
            {"cx": 3.0,"cy":0.0,"w":1.0,"h":9.0},
        ])
        self.get_logger().info('RRT* obstacles: 3 vertical shelf racks')

    def _update_obstacles(self):
        # The planner sees ONLY what the costmap contains:
        # the static map plus whatever the lidar has actually sensed.
        self.rrt.set_obstacles(self.costmap.as_obstacle_list())

    def _spawn_world_obstacle(self, p0, p1, frac=0.5):
        """
        Place an obstacle in the WORLD on the segment p0->p1.
        The robot is NOT told. It will only discover this if its lidar
        rays hit it, which requires driving within sensor range.
        """
        x0,y0=p0; x1,y1=p1
        dx,dy = x1-x0, y1-y0
        d = math.hypot(dx,dy)
        if d < 0.5: return False

        best=None
        for k in range(0,11):
            for sgn in (0,1,-1):
                f = frac + sgn*(k*0.04)
                if f<0.20 or f>0.85: continue
                cx = x0 + dx*f
                cy = y0 + dy*f
                if abs(cx)>7.0 or abs(cy)>5.2: continue
                inside=False
                for o in self.costmap.static_layer:
                    if (abs(cx-o["cx"])<o["w"]/2+0.85 and
                        abs(cy-o["cy"])<o["h"]/2+0.3):
                        inside=True; break
                if not inside:
                    best=(cx,cy); break
            if best: break
        if not best: return False

        ox,oy=best
        self.world_obstacles.append(
            {"cx":ox,"cy":oy,"w":2.2,"h":1.6,
             "ttl":40.0,"created":time.time()})
        self.get_logger().info(
            f'[WORLD] obstacle exists at ({ox:.1f},{oy:.1f}) '
            f'- robot has NOT sensed it yet')
        # Publish for the 3D view (the world renders it; the robot is blind to it)
        m=String(); m.data=json.dumps(self.world_obstacles)
        self.obstacle_pub.publish(m)
        return True

    def _sense(self):
        """
        Run one lidar scan from the current pose. Fold returns into the
        costmap. Returns True if the costmap gained anything new, i.e.
        the robot has just DETECTED something it did not know about.
        """
        ranges, hits = self.lidar.scan(
            self.robot_x, self.robot_y, self.heading,
            self.costmap.static_layer + self.world_obstacles)

        # publish scan for visualisation (rate limited)
        now = time.time()
        if now - self._last_scan_pub > 0.12:
            self._last_scan_pub = now
            sm=String()
            sm.data=json.dumps({
                "x":round(self.robot_x,3),"y":round(self.robot_y,3),
                "heading":round(self.heading,3),
                "fov":round(math.degrees(self.lidar.fov),1),
                "max_range":self.lidar.max_range,
                "ranges":[round(r,2) for r in ranges],
                "hits":[[round(h[0],2),round(h[1],2)] for h in hits],
                "sensed":len(self.costmap.obstacle_layer)
            })
            self.scan_pub.publish(sm)

        newly_seen = self.costmap.integrate_scan(hits)
        if newly_seen:
            self._update_obstacles()
        return newly_seen

    def _path_blocked(self, traj, from_idx, lookahead=45):
        """Is the upcoming section of the trajectory now in collision?"""
        for j in range(from_idx, min(from_idx+lookahead, len(traj))):
            px,py,_ = traj[j]
            if not self.rrt._collision_free((px,py)):
                return True
        return False

    def _inject_obstacle(self):
        ay=random.choice([-4.5,4.5]); ox=random.uniform(-5,5)
        obs={"cx":ox,"cy":ay,"w":0.8,"h":0.8,"ttl":20.0,"created":time.time()}
        self.dynamic_obstacles.append(obs)
        self._update_obstacles()
        m=String(); m.data=json.dumps(self.dynamic_obstacles); self.obstacle_pub.publish(m)
        self.get_logger().info(f'Dynamic obstacle at ({ox:.1f},{ay:.1f})')

    def _cleanup_obstacles(self):
        now=time.time(); before=len(self.world_obstacles)
        self.world_obstacles=[o for o in self.world_obstacles
                              if now-o["created"]<o["ttl"]]
        if len(self.world_obstacles)<before:
            # object removed from the world; forget what we sensed of it
            self.costmap.clear_obstacle_layer()
            self._update_obstacles()
            m=String(); m.data=json.dumps(self.world_obstacles)
            self.obstacle_pub.publish(m)

    def scan_result_callback(self,msg):
        self.scan_result=json.loads(msg.data)
        self.scan_received=True
        # Remember exactly WHICH bin the perception identified
        self._detected_bin_id = self.scan_result.get('bin_id','')

    def waypoints_callback(self,msg):
        if self.is_executing: return
        wps=json.loads(msg.data)
        self.get_logger().info(f'Received {len(wps)} waypoints')
        self.publish_status(f'Starting: {len(wps)} steps')
        import threading
        threading.Thread(target=self._execute,args=(wps,),daemon=True).start()

    def _execute(self,waypoints):
        self.is_executing=True
        self._scan_failed=False
        self._midleg_done=False
        self._start_blocked=False
        total=len(waypoints)

        # Place an obstacle in the WORLD on the route this mission will use.
        # The robot is not informed — it must sense it with the lidar.
        navs=[w for w in waypoints if w.get('action')=='navigate']
        prev=(self.robot_x,self.robot_y)
        legs=[]
        for w in navs:
            gx=float(w.get('world_x',0.0)); gy=float(w.get('world_y',0.0))
            legs.append((math.hypot(gx-prev[0],gy-prev[1]), prev, (gx,gy)))
            prev=(gx,gy)
        # block the LONGEST leg - most visible, most room to reroute
        legs=[l for l in legs if l[0] > 3.0]
        if legs:
            legs.sort(key=lambda l: -l[0])
            self._spawn_world_obstacle(legs[0][1], legs[0][2], frac=0.5)

        for i,wp in enumerate(waypoints):
            action=wp.get('action','navigate')
            desc=wp.get('description',action)
            wx=max(-7.5,min(7.5,float(wp.get('world_x',0.0))))
            wy=max(-5.5,min(5.5,float(wp.get('world_y',0.0))))

            self.get_logger().info(f'Step {i+1}/{total}: {desc}')
            self.publish_status(f'Step {i+1}/{total}: {desc}')
            self._cleanup_obstacles()

            if action=='navigate':
                self._navigate(wx,wy)

            elif action=='scan':
                # Autonomous scan — robot detects bin position itself
                color=wp.get('color','red')
                shelf_x=wx
                self._last_scan_color=color
                self._last_scan_shelf=wx + 1.0  # actual shelf x (approach was -1.0)
                self.publish_status(f'Step {i+1}/{total}: Scanning shelf for {color} bin')
                detected_z=self._scan_for_bin(shelf_x, color)

                if detected_z is not None:
                    self.get_logger().info(f'Detected {color} bin at z={detected_z:.2f}')
                    self.publish_status(f'Step {i+1}/{total}: {color} bin found at z={detected_z:.1f}')
                    # Robot BODY stays in aisle at approach_x (=shelf_x here).
                    # Only Y changes to line up with the bin. The ARM reaches
                    # the remaining 1m into the shelf — body never enters shelf.
                    self._navigate(shelf_x, detected_z)
                else:
                    self.get_logger().warning(
                        f'{color} bin not found on this shelf — aborting pick')
                    self.publish_status(
                        f'Step {i+1}/{total}: {color} bin NOT FOUND')
                    self._detected_bin_id = ''
                    self._scan_failed = True

            elif action=='pick':
                color=wp.get('color','object')
                if getattr(self,'_scan_failed',False):
                    self.get_logger().warning(
                        'Skipping pick — perception did not find the bin')
                    self.publish_status(
                        f'Step {i+1}/{total}: Skipped (bin not detected)')
                    self._scan_failed=False
                    continue
                self.publish_status(f'Step {i+1}/{total}: Picking {color} bin')
                self._arm('pick',self.robot_x,self.robot_y)
                time.sleep(4.0)   # allow full reach-grip-lift motion
                # Report the pick so the world (image_mapper) removes this bin
                shelf_x = getattr(self,'_last_scan_shelf', self.robot_x+1.0)
                bin_id  = getattr(self,'_detected_bin_id','')
                pk=String()
                # bin_id tells the browser EXACTLY which mesh to remove.
                # No matching logic in the browser — the backend decides.
                pk.data=json.dumps({"shelf_x":shelf_x,"color":color,
                                    "bin_id":bin_id})
                self.bin_picked_pub.publish(pk)
                self.get_logger().info(f'Bin picked & removed from world: {color} @ shelf x={shelf_x:.1f}')

            elif action=='place':
                self.publish_status(f'Step {i+1}/{total}: Placing bin')
                self._arm('place',wx,wy)
                time.sleep(4.0)   # allow full extend-lower-release motion

        self.publish_status('Task complete')
        self.is_executing=False

    def _scan_for_bin(self, shelf_x, color, timeout=8.0):
        """
        Request a scan from image_mapper and wait for the detected
        bin position. Returns the Z coordinate of the bin, or None.
        This is the autonomous perception step.
        """
        self.scan_received=False
        self.scan_result=None

        req=String()
        req.data=json.dumps({
            "shelf_x": shelf_x,
            "color":   color,
            "robot_x": self.robot_x,
            "robot_y": self.robot_y
        })
        self.scan_req_pub.publish(req)

        # Wait for image_mapper to respond
        start=time.time()
        while not self.scan_received and (time.time()-start)<timeout:
            time.sleep(0.05)

        if self.scan_received and self.scan_result:
            if self.scan_result.get('found'):
                return float(self.scan_result.get('z', 0.0))
        return None

    def _navigate(self,tx,ty):
        dist=math.hypot(tx-self.robot_x,ty-self.robot_y)
        if dist<0.15: return

        for _attempt in range(6):
          interrupted = False

          # RECOVERY: if we've failed several times the costmap may hold
          # stale returns. Clear the obstacle layer and re-sense (Nav2's
          # 'clear costmap' recovery behaviour).
          if _attempt == 3:
              n = self.costmap.clear_obstacle_layer()
              self._update_obstacles()
              self.get_logger().warning(
                  f'[RECOVERY] cleared {n} sensed obstacles, re-sensing')
              self.publish_status('RECOVERY: clearing costmap')
              self._sense()
              time.sleep(0.3)
          self.get_logger().info(f'RRT* planning: ({self.robot_x:.2f},{self.robot_y:.2f}) → ({tx:.2f},{ty:.2f})')
          path=self.rrt.plan((self.robot_x,self.robot_y),(tx,ty))
          self.get_logger().info(f'RRT* path: {len(path)} waypoints')

          pm=String(); pm.data=json.dumps([{"x":p[0],"y":p[1]} for p in path]); self.path_pub.publish(pm)

          # Publish the RRT* search tree (subsampled) for the schematic view
          edges = self.rrt.tree_edges
          if len(edges) > 900:
              step = max(1, len(edges)//900)
              edges = edges[::step]
          tm=String()
          tm.data=json.dumps({
              "edges":[[round(a,2),round(b,2),round(cc,2),round(d,2)]
                       for a,b,cc,d in edges],
              "obstacles":[{"cx":o["cx"],"cy":o["cy"],"w":o["w"],"h":o["h"]}
                           for o in self.rrt.obstacles],
              "radius": self.rrt.ROBOT_RADIUS,
              "nodes": len(self.rrt.tree_edges)
          })
          self.tree_pub.publish(tm)

          traj=self.topp.generate(path)
          if not traj: return

          t0=time.time()
          for _i,(px,py,pt) in enumerate(traj):
              el=time.time()-t0; wait=pt-el
              if wait>0: time.sleep(min(wait,0.05))

              prev_x, prev_y = self.robot_x, self.robot_y

              self.noise_t+=self.noise_speed*0.05
              nx=self.noise_x.noise(self.noise_t)*self.noise_amp
              ny=self.noise_y.noise(self.noise_t+100)*self.noise_amp
              cx=px+nx; cy=py+ny
              if self.rrt._collision_free((cx,cy)): self.robot_x=cx; self.robot_y=cy
              else: self.robot_x=px; self.robot_y=py
              drift=math.hypot(self.robot_x-px,self.robot_y-py)
              if drift>0.12:
                  self.robot_x+=(px-self.robot_x)*0.3
                  self.robot_y+=(py-self.robot_y)*0.3

              # heading from actual motion (lidar points where we travel)
              hdx, hdy = self.robot_x-prev_x, self.robot_y-prev_y
              if math.hypot(hdx,hdy) > 1e-4:
                  self.heading = math.atan2(hdy, hdx)

              # ── PERCEPTION ──
              # Scan every few control cycles. The robot discovers obstacles
              # only when rays actually return off them.
              if _i % 2 == 0:
                  self._sense()
                  # Replan whenever the path AHEAD is in collision against
                  # the sensed costmap. Gating on "newly detected" was wrong:
                  # the first return often arrives while the blockage is
                  # still beyond the lookahead, and by the time the robot is
                  # close the detection is no longer new.
                  if self._path_blocked(traj, _i):
                      self.get_logger().warning(
                          f'[LIDAR] obstacle in path at '
                          f'<= {self.lidar.max_range}m - replanning')
                      self.publish_status('OBSTACLE SENSED - replanning')
                      interrupted = True
                      break

          if interrupted:
            time.sleep(0.5)
            continue          # replan from where the robot actually is

          self.robot_x=tx; self.robot_y=ty
          return

    def _arm(self,action,x,y):
        c=String(); c.data=json.dumps({"action":action,"x":x,"y":y}); self.arm_pub.publish(c)

    def publish_position(self):
        m=String(); m.data=json.dumps({"x":round(self.robot_x,3),"y":round(self.robot_y,3)}); self.position_pub.publish(m)

    def publish_status(self,s):
        m=String(); m.data=s; self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node=NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()