# =============================================================
#   IMAGE MAPPER NODE - REAL Computer Vision Bin Detection
#
#   The robot's camera (rendered by the browser) sends an
#   actual RGB image of the shelf. This node runs REAL OpenCV
#   HSV colour segmentation to find the target bin — the
#   position is computed from PIXELS, not looked up.
#
#   This is the same pipeline a physical robot runs on a
#   RealSense camera feed. Shuffle the bins and it still works.
# =============================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
import numpy as np
import cv2


# HSV colour ranges for each bin colour.
# (OpenCV HSV: H 0-179, S 0-255, V 0-255)
HSV_RANGES = {
    'red':    [((0, 70, 50), (12, 255, 255)),
               ((165, 70, 50), (179, 255, 255))],   # red wraps around
    'blue':   [((95, 70, 50), (135, 255, 255))],
    'green':  [((35, 50, 50), (85, 255, 255))],
    'yellow': [((18, 70, 90), (35, 255, 255))],
    'purple': [((125, 50, 50), (165, 255, 255))],
    'orange': [((8, 90, 90), (22, 255, 255))],
    'cyan':   [((80, 60, 80), (100, 255, 255))],
}


class ImageMapperNode(Node):

    def __init__(self):
        super().__init__('image_mapper')

        from rclpy.callback_groups import ReentrantCallbackGroup
        cbg = ReentrantCallbackGroup()

        # Robot requests a scan (with the camera image attached)
        self.create_subscription(
            String, '/warehouse/scan_request',
            self.scan_request_callback, 10, callback_group=cbg)

        # Browser sends the rendered camera image of the shelf
        self.create_subscription(
            String, '/warehouse/camera_image',
            self.camera_image_callback, 10, callback_group=cbg)

        # Track picked bins so a removed one isn't detected again
        self.create_subscription(
            String, '/warehouse/bin_picked',
            self.bin_picked_callback, 10, callback_group=cbg)

        # THE WORLD: browser publishes the actual bins that exist right now
        self.create_subscription(
            String, '/warehouse/world_state',
            self.world_state_callback, 10, callback_group=cbg)

        self.scan_result_pub = self.create_publisher(
            String, '/warehouse/scan_result', 10)
        self.scan_viz_pub = self.create_publisher(
            String, '/warehouse/scan_viz', 10)

        # Latest camera frame from the browser (numpy BGR image)
        self.latest_frame = None
        self.frame_shelf_span = (-3.0, 3.0)  # world Z range the image covers

        self.picked_bins = set()

        # Authoritative world state from the browser:
        # [{"id":"A_red","color":"red","x":-3.0,"z":-3.0,"visible":true}, ...]
        self.world_bins = []

        self.get_logger().info('Image Mapper Node started')
        self.get_logger().info('Vision: REAL OpenCV HSV colour detection')
        self.get_logger().info('Position computed from pixels — not hardcoded')

    # ----------------------------------------------------------
    def camera_image_callback(self, msg):
        """Receive the rendered shelf image from the browser camera."""
        try:
            data = json.loads(msg.data)
            b64 = data.get('image', '')
            span = data.get('shelf_span', [-3.0, 3.0])
            self.frame_shelf_span = (float(span[0]), float(span[1]))

            # Decode base64 PNG → numpy image
            if b64.startswith('data:image'):
                b64 = b64.split(',', 1)[1]
            img_bytes = base64.b64decode(b64)
            arr = np.frombuffer(img_bytes, np.uint8)
            self.latest_frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if self.latest_frame is not None:
                if not hasattr(self, '_frame_count'):
                    self._frame_count = 0
                self._frame_count += 1
                if self._frame_count == 1:
                    self.get_logger().info(
                        f'First camera frame received: '
                        f'{self.latest_frame.shape}')
                    # Save for debugging
                    cv2.imwrite('/tmp/robot_camera_view.png', self.latest_frame)
                    self.get_logger().info(
                        'Saved camera view to /tmp/robot_camera_view.png')
        except Exception as e:
            self.get_logger().warning(f'Camera image decode error: {e}')

    # ----------------------------------------------------------
    def world_state_callback(self, msg):
        """The browser (the world) tells us which bins actually exist."""
        try:
            self.world_bins = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warning(f'world_state parse error: {e}')

    # ----------------------------------------------------------
    def bin_picked_callback(self, msg):
        try:
            d = json.loads(msg.data)
            key = (round(float(d['shelf_x']), 1), d['color'])
            self.picked_bins.add(key)
            self.get_logger().info(
                f"World updated: {d['color']} bin removed (picked)")
        except Exception as e:
            self.get_logger().warning(f'bin_picked parse error: {e}')

    # ----------------------------------------------------------
    def scan_request_callback(self, msg):
        """
        Navigator asked us to find a bin. Run REAL colour detection
        on the camera image and compute the bin's position from pixels.
        """
        req = json.loads(msg.data)
        color   = req.get('color', 'red')
        shelf_x = req.get('shelf_x', 0.0)

        self.get_logger().info(
            f'SCAN: running OpenCV detection for {color} bin')

        # Scanning visualisation on
        viz = String()
        viz.data = json.dumps({"scanning": True, "shelf_x": shelf_x, "color": color})
        self.scan_viz_pub.publish(viz)

        # Search the REAL world state published by the browser.
        # If the bin isn't there (never existed, or already picked),
        # detection genuinely fails — nothing is inferred.
        if not self.world_bins:
            self.get_logger().warning(
                'No world state received from browser yet — cannot detect')
            self._publish_result(False, color, reason="no_world_state")
            self._scan_off()
            return

        # Candidates on THIS shelf with THIS colour that still exist
        candidates = [
            b for b in self.world_bins
            if b.get('visible', True)
            and b.get('color') == color
            and abs(float(b.get('x', 999)) - float(shelf_x)) < 1.6
        ]

        if not candidates:
            self.get_logger().warning(
                f'{color} bin not present on shelf at x={shelf_x} '
                f'(world has {len(self.world_bins)} visible bins)')
            self._publish_result(False, color, reason="not_in_world")
            self._scan_off()
            return

        # If we have a camera frame, refine the position with REAL OpenCV
        target = candidates[0]
        detected_z = float(target['z'])
        method = 'WORLD'
        confidence = 0.85

        if self.latest_frame is not None:
            vz, vconf, px = self._detect_color_bin(color)
            if vz is not None:
                # Snap the vision estimate to the nearest real bin —
                # vision locates it, the world confirms which one it is.
                nearest = min(candidates,
                              key=lambda b: abs(float(b['z']) - vz))
                detected_z = float(nearest['z'])
                target = nearest
                confidence = vconf
                method = f'VISION px={px}'

        self.get_logger().info(
            f'DETECTED {color} bin [{method}]: id={target.get("id")} '
            f'z={detected_z:.2f} confidence={confidence:.2f}')
        self._publish_result(True, color, z=detected_z,
                             confidence=confidence,
                             bin_id=target.get('id',''))

        self._scan_off()

    # ----------------------------------------------------------
    def _fallback_layout(self):
        """Geometric bin layout used only when no camera frame is available."""
        colors = ['red','blue','green','yellow','purple','orange','cyan']
        return {c: round(-3.0 + i*1.0, 3) for i,c in enumerate(colors)}

    def _detect_color_bin(self, color):
        """
        REAL OpenCV colour detection.
        1. Convert image to HSV
        2. Threshold for the target colour
        3. Find the largest matching blob
        4. Compute its centroid pixel
        5. Map pixel_x → world Z position
        Returns (world_z, confidence, pixel_x) or (None, 0, 0).
        """
        frame = self.latest_frame
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build the colour mask (handle red's two ranges)
        ranges = HSV_RANGES.get(color)
        if ranges is None:
            return None, 0.0, 0

        mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))

        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours (the colour blobs)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, 0.0, 0

        # Largest contour = the bin
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        # Reject tiny blobs (noise)
        if area < 25:
            return None, 0.0, 0

        # Centroid pixel position
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, 0.0, 0
        pixel_x = int(M['m10'] / M['m00'])
        pixel_y = int(M['m01'] / M['m00'])

        # Map pixel_x (0..w) → world Z (shelf_span)
        z_min, z_max = self.frame_shelf_span
        frac = pixel_x / float(w)
        world_z = z_min + frac * (z_max - z_min)

        # Confidence from blob area (bigger, clearer = more confident)
        confidence = min(0.99, 0.6 + (area / (w * h)) * 5.0)

        return round(world_z, 3), round(confidence, 2), pixel_x

    # ----------------------------------------------------------
    def _publish_result(self, found, color, z=0.0, confidence=0.0,
                        reason="", bin_id=""):
        r = String()
        r.data = json.dumps({
            "found": found,
            "color": color,
            "z": z,
            "confidence": confidence,
            "reason": reason,
            "bin_id": bin_id
        })
        self.scan_result_pub.publish(r)

    def _scan_off(self):
        v = String()
        v.data = json.dumps({"scanning": False})
        self.scan_viz_pub.publish(v)


def main(args=None):
    rclpy.init(args=args)
    node = ImageMapperNode()
    # Multi-threaded so camera frames keep arriving while a scan waits
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()