# =============================================================
#   PYBULLET BRIDGE NODE
#   Subscribes to ROS2 navigation commands
#   Drives the Franka Panda robot in PyBullet
#   Shows 3D robot moving in real time
# =============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import os

# Add your existing project to path
sys.path.insert(0, '/mnt/c/Users/prajj/ur5_ai_robot')

import pybullet as p
import pybullet_data
import numpy as np
import time


class PyBulletBridgeNode(Node):

    def __init__(self):
        super().__init__('pybullet_bridge')

        # Subscribe to arm commands from navigator
        self.create_subscription(
            String, '/warehouse/arm_command',
            self.arm_command_callback, 10)

        # Subscribe to robot position
        self.create_subscription(
            String, '/warehouse/robot_position',
            self.position_callback, 10)

        # Subscribe to status
        self.create_subscription(
            String, '/warehouse/status',
            self.status_callback, 10)

        # Init PyBullet
        self.setup_pybullet()

        # Timer — step simulation at 60Hz
        self.create_timer(1.0/60.0, self.step_simulation)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.holding = False

        self.get_logger().info('PyBullet Bridge started')

    def setup_pybullet(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0.5, 0, 0]
        )

        # Load ground plane
        p.loadURDF("plane.urdf")

        # Load Franka Panda
        self.robot = p.loadURDF(
            "franka_panda/panda.urdf",
            basePosition=[0, 0, 0],
            useFixedBase=True
        )

        # Add warehouse objects
        self.setup_warehouse()

        # Home position
        home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04]
        for i, angle in enumerate(home):
            p.resetJointState(self.robot, i, angle)

        self.get_logger().info('PyBullet scene ready')

    def setup_warehouse(self):
        # Table
        table_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.4, 0.5, 0.05])
        table_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.4, 0.5, 0.05],
            rgbaColor=[0.55, 0.35, 0.15, 1.0])
        p.createMultiBody(0, table_col, table_vis,
                         basePosition=[0.45, 0, 0.05])

        # Red block — pick target
        box_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
        box_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025],
            rgbaColor=[1, 0, 0, 1])
        self.red_block = p.createMultiBody(
            0.1, box_col, box_vis,
            basePosition=[0.40, 0.0, 0.125])

        # Dispatch zone marker
        disp_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.002])
        disp_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.002],
            rgbaColor=[0, 0.8, 0, 1])
        p.createMultiBody(0, disp_col, disp_vis,
                         basePosition=[0.60, 0.0, 0.102])

        p.addUserDebugText(
            "DISPATCH", [0.60, 0.0, 0.20],
            textColorRGB=[0, 1, 0], textSize=1.2)

    def arm_command_callback(self, msg):
        cmd = json.loads(msg.data)
        action = cmd.get('action', '')

        if action == 'pick':
            self.get_logger().info('PyBullet: executing pick')
            self.animate_pick()
        elif action == 'place':
            self.get_logger().info('PyBullet: executing place')
            self.animate_place()

    def animate_pick(self):
        # Move arm down to block
        pick_pos = [0.40, 0.0, 0.15]
        self.move_arm_to(pick_pos)
        time.sleep(0.5)
        self.holding = True
        self.get_logger().info('PyBullet: picked')

    def animate_place(self):
        # Move arm to dispatch zone
        place_pos = [0.60, 0.0, 0.20]
        self.move_arm_to(place_pos)
        time.sleep(0.5)
        self.holding = False
        self.get_logger().info('PyBullet: placed')

    def move_arm_to(self, target_pos):
        joint_poses = p.calculateInverseKinematics(
            self.robot, 11, target_pos)
        for i in range(7):
            p.setJointMotorControl2(
                self.robot, i,
                p.POSITION_CONTROL,
                targetPosition=joint_poses[i],
                force=500,
                maxVelocity=1.0
            )
        # Step simulation to animate
        for _ in range(120):
            p.stepSimulation()
            time.sleep(1.0/120.0)

    def position_callback(self, msg):
        pos = json.loads(msg.data)
        self.robot_x = pos['x']
        self.robot_y = pos['y']

    def status_callback(self, msg):
        p.addUserDebugText(
            msg.data,
            [0.0, 0.0, 0.8],
            textColorRGB=[1, 1, 0],
            textSize=1.0,
            lifeTime=3.0
        )

    def step_simulation(self):
        p.stepSimulation()


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()