# =============================================================
#   TASK PLANNER NODE - Claude AI decomposes orders
#   Robot body stays in AISLE at offset from shelf.
#   The ARM reaches ~1m into the shelf to grab bins.
#   Robot discovers bin position via scan (no hardcoded z).
# =============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import anthropic

# Robot stays this far from shelf centre — arm bridges the gap
ARM_REACH = 1.0


class TaskPlannerNode(Node):

    def __init__(self):
        super().__init__('task_planner')
        self.create_subscription(String,'/warehouse/order',self.order_callback,10)
        self.waypoints_pub=self.create_publisher(String,'/warehouse/waypoints',10)
        self.status_pub=self.create_publisher(String,'/warehouse/status',10)
        self.client=anthropic.Anthropic(api_key=os.getenv('ANTHROPIC_API_KEY',''))
        self.get_logger().info('Task Planner Node started')
        self.get_logger().info('Send orders to /warehouse/order topic')

    def order_callback(self,msg):
        order=msg.data
        self.get_logger().info(f'Order received: {order}')
        self.publish_status(f'Planning: {order}')
        wps=self.plan_with_claude(order)
        if wps:
            m=String(); m.data=json.dumps(wps); self.waypoints_pub.publish(m)
            self.publish_status(f'Starting: {len(wps)} steps')
            self.get_logger().info(f'Published {len(wps)} waypoints')
        else:
            self.publish_status('Planning failed')

    def plan_with_claude(self,order):
        try:
            prompt=f"""You are a warehouse robot task planner.

WAREHOUSE LAYOUT:
- 3 vertical shelves along the Y axis
- Shelf A at x=-3.0, Shelf B at x=0.0, Shelf C at x=3.0
- Each shelf runs y=-4.5 to y=4.5, shelves are SOLID obstacles

AISLES (safe corridors):
- Bottom aisle y=-5.0, Top aisle y=5.0
- Approach lanes beside shelves at x=-4.0, x=-1.5, x=1.5, x=4.0

KEY POSITIONS:
- Home:      x=-6.0, y=-5.0
- Pick Zone: x=-6.0, y=5.0
- Dispatch:  x=6.0,  y=5.0

ROBOT ARM OFFSET (CRITICAL):
The robot BODY must stay in the aisle beside the shelf — NEVER
inside the shelf. The robot has a 1m reach arm. So the robot
approaches at x = shelf_x - 1.0 and the arm extends to the bin.
- Shelf A (x=-3.0): robot approaches at x=-4.0
- Shelf B (x=0.0):  robot approaches at x=-1.0
- Shelf C (x=3.0):  robot approaches at x=2.0
Use these approach_x values for ALL scan and pick positions.

AUTONOMOUS VISION:
Robot does NOT know bin positions in advance. It SCANS the shelf
with its camera, detects the colour bin, and auto-moves to it
(staying at approach_x, only changing y).

ACTIONS:
- navigate: move to (world_x, world_y) via aisles
- scan:     scan shelf for colour; robot auto-moves along the
            approach lane to the detected bin (include "color")
- pick:     arm extends to grab the bin (include "color")
- place:    place held bin at (world_x, world_y)

RULES:
1. Travel via aisles y=-5 or y=5 to change x
2. Never cross through x=-3, 0, 3 (solid shelves)
3. Approach shelf via its approach_x lane
4. scan/pick world_x = approach_x (NOT shelf_x)

ORDER: {order}

Return ONLY a JSON array. Example for "pick red bin from shelf A and deliver to dispatch":
[
  {{"step":1,"action":"navigate","description":"Go to bottom aisle at Shelf A approach","world_x":-4.0,"world_y":-5.0}},
  {{"step":2,"action":"navigate","description":"Enter Shelf A approach lane","world_x":-4.0,"world_y":-4.0}},
  {{"step":3,"action":"scan","description":"Scan Shelf A for red bin","world_x":-4.0,"world_y":0.0,"color":"red"}},
  {{"step":4,"action":"pick","description":"Arm extends to pick red bin","world_x":-4.0,"world_y":0.0,"color":"red"}},
  {{"step":5,"action":"navigate","description":"Exit to top aisle","world_x":-4.0,"world_y":5.0}},
  {{"step":6,"action":"navigate","description":"Go to dispatch","world_x":6.0,"world_y":5.0}},
  {{"step":7,"action":"place","description":"Place at dispatch","world_x":6.0,"world_y":5.0}}
]

Return ONLY the JSON array, no other text."""

            resp=self.client.messages.create(
                model="claude-haiku-4-5-20251001",
                max_tokens=1500,
                messages=[{"role":"user","content":prompt}])
            raw=resp.content[0].text.strip().replace('```json','').replace('```','').strip()
            wps=json.loads(raw)
            self.get_logger().info(f'Claude planned {len(wps)} steps')
            return wps
        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            return self.fallback(order)

    def fallback(self,order):
        o=order.lower()
        if 'shelf a' in o: sx,label=-3.0,'Shelf A'
        elif 'shelf c' in o: sx,label=3.0,'Shelf C'
        else: sx,label=0.0,'Shelf B'

        # Robot approaches from the left side of the shelf, offset by arm reach
        approach_x = round(sx - ARM_REACH, 2)

        color='red'
        for c in ['red','blue','green','yellow','purple','orange','cyan']:
            if c in o: color=c; break

        return [
            {"step":1,"action":"navigate","description":f"Go to bottom aisle at {label} approach","world_x":approach_x,"world_y":-5.0},
            {"step":2,"action":"navigate","description":f"Enter {label} approach lane","world_x":approach_x,"world_y":-4.0},
            {"step":3,"action":"scan","description":f"Scan {label} for {color} bin","world_x":approach_x,"world_y":0.0,"color":color},
            {"step":4,"action":"pick","description":f"Arm extends to pick {color} bin","world_x":approach_x,"world_y":0.0,"color":color},
            {"step":5,"action":"navigate","description":"Exit to top aisle","world_x":approach_x,"world_y":5.0},
            {"step":6,"action":"navigate","description":"Go to dispatch","world_x":6.0,"world_y":5.0},
            {"step":7,"action":"place","description":"Place at dispatch","world_x":6.0,"world_y":5.0},
        ]

    def publish_status(self,s):
        m=String(); m.data=s; self.status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node=TaskPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()