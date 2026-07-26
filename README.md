# Autonomous Warehouse Robot — ROS2

Natural-language task planning, RRT* motion planning, and lidar-based
obstacle avoidance for a simulated warehouse robot.

Give it an order in plain English and it plans the route, navigates the
aisles, identifies the target bin by colour, picks it, and delivers it.
## Demo

[![Watch the demo](https://img.youtube.com/vi/k35GhiyE4QM/maxresdefault.jpg)](https://youtu.be/k35GhiyE4QM)

Natural-language order → RRT* planning → lidar obstacle detection → pick and deliver.

## Architecture

Four ROS2 nodes:

| Node | Role |
|------|------|
| `task_planner` | LLM decomposes natural language into waypoints |
| `navigator` | RRT* global planning, TOPP-RA trajectories, lidar sensing |
| `image_mapper` | OpenCV HSV colour segmentation for bin identification |
| `dashboard_server` | Three.js 3D view + 2D planner schematic |

## Motion planning

- **RRT\*** — sampling-based global planner over a layered costmap
- **TOPP-RA** — time-optimal trajectory generation with velocity and
  acceleration limits
- **Costmap** — static layer (racking), obstacle layer (sensed returns),
  inflation by robot radius, following the Nav2 model
- **Recovery behaviours** — clear the obstacle layer and re-sense on
  repeated planning failure

## Perception

- Simulated 2D lidar: 270° FOV, 3.5 m range, 90 rays, with range noise
- Obstacles are not known in advance — the robot discovers them only when
  rays return off them, then replans
- OpenCV HSV segmentation for bin colour identification

## Running

```bash
export ANTHROPIC_API_KEY="your-key"
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch warehouse_robot warehouse.launch.py
```

Dashboard at `http://localhost:8080`

## Limitations

The visualisation is kinematic, not physics-based. Three.js has no contact
dynamics, so grasping is animated rather than simulated. Physics-based
grasping with contact forces and slip is planned via PyBullet.

## Roadmap

- Physics-based grasping (PyBullet)
- Local planner for reactive avoidance between global replans
- PLC safety layer: deterministic interlocks holding veto over autonomy

## Requirements

ROS2 Jazzy · Python 3.12 · opencv-python-headless · numpy · anthropic
