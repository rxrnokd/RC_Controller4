"""Run with ROS Jazzy's system Python INSIDE the existing container."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster, TransformException


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class Bridge(Node):
    def __init__(self, config_path):
        super().__init__('rc_local_bridge')
        self.c = json.loads(config_path.read_text())
        self.socket_path = str(config_path.parent / self.c['ros']['socket'])
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = -math.inf
        self.last_host_stamp = None
        self.last_scan = -math.inf
        self.last_odom = -math.inf
        self.map_message = None
        self.status = {'type': 'ros_status', 'tracking': False, 'pose': None}
        self.done = threading.Event()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, qos)
        laser = self.c['ros']['laser_xyz_yaw']
        if laser is not None:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = 'base_link', 'laser'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, laser[:3])
            t.transform.rotation.z, t.transform.rotation.w = math.sin(laser[3]/2), math.cos(laser[3]/2)
            self.static_tf.sendTransform(t)
        self.create_timer(0.05, self.tick)
        self.worker = threading.Thread(target=self.transport, daemon=True)
        self.worker.start()

    def scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (msg.header.frame_id == 'laser' and 0 <= age < self.c['ros']['freshness_s'] and
                any(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges)):
            self.last_scan = time.monotonic()

    def map_received(self, msg):
        if msg.header.frame_id != 'map' or len(msg.data) != msg.info.width * msg.info.height:
            return
        if len(msg.data) > self.c['ros']['max_map_cells']:
            self.get_logger().error('Map too large for configured max_map_cells')
            return
        if not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0:
            return
        raw = bytes(v + 1 for v in msg.data)
        origin = msg.info.origin
        metadata = {'width': msg.info.width, 'height': msg.info.height, 'resolution': msg.info.resolution,
                    'origin': [origin.position.x, origin.position.y, yaw(origin.orientation)], 'frame': 'map'}
        revision = hashlib.sha256(raw + json.dumps(metadata, sort_keys=True).encode()).hexdigest()
        with self.lock:
            self.map_message = dict(metadata, v=1, type='map', revision=revision,
                                    encoding='base64_uint8_occupancy_plus_one', data=base64.b64encode(raw).decode())

    def tick(self):
        now = time.monotonic()
        with self.lock:
            state, rx = self.latest, self.received_at
        if state and now - rx < 0.3 and now - state['server_time'] < 0.3:
            od = state.get('odom')
            if od and od.get('valid') and state['server_time'] != self.last_host_stamp:
                self.last_host_stamp = state['server_time']
                # Same physical host, monotonic time accounts for transport delay.
                delay = max(0, now - state['server_time'])
                stamp = Time(nanoseconds=self.get_clock().now().nanoseconds - int(delay * 1e9)).to_msg()
                msg = Odometry()
                msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'odom', 'base_link'
                msg.pose.pose.position.x, msg.pose.pose.position.y = od['x'], od['y']
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(od['yaw']/2), math.cos(od['yaw']/2)
                msg.twist.twist.linear.x, msg.twist.twist.angular.z = od['speed_m_s'], od['yaw_rate']
                # Conservative placeholders for uncertainty, not invented measurements.
                for i, value in enumerate((0.05, 0.05, 1e6, 1e6, 1e6, 0.1)):
                    msg.pose.covariance[i*7] = value
                    msg.twist.covariance[i*7] = value
                self.odom_pub.publish(msg)
                t = TransformStamped()
                t.header, t.child_frame_id = msg.header, msg.child_frame_id
                t.transform.translation.x, t.transform.translation.y = od['x'], od['y']
                t.transform.rotation = msg.pose.pose.orientation
                self.tf.sendTransform(t)
                self.last_odom = now
        ready = now-self.last_scan < self.c['ros']['freshness_s'] and now-self.last_odom < 0.3
        status = {'v': 1, 'type': 'ros_status', 'tracking': False, 'pose': None,
                  'scan_fresh': now-self.last_scan < self.c['ros']['freshness_s'],
                  'odom_fresh': now-self.last_odom < 0.3, 'reason': 'scan / odom / map / TF unavailable'}
        try:
            if ready and self.map_message:
                transform = self.buffer.lookup_transform('map', 'base_link', Time())
                correction = self.buffer.lookup_transform('map', 'odom', Time())
                ros_now = self.get_clock().now().nanoseconds / 1e9
                ages = [ros_now - (t.header.stamp.sec + t.header.stamp.nanosec / 1e9) for t in (transform, correction)]
                if all(-0.5 <= a < self.c['ros']['freshness_s'] for a in ages):
                    p = transform.transform
                    status.update(tracking=True, reason='fresh scan, odom, map and TF; quality not certified',
                                  pose={'x': p.translation.x, 'y': p.translation.y, 'yaw': yaw(p.rotation)})
        except TransformException:
            pass
        with self.lock:
            self.status = status

    def transport(self):
        while not self.done.is_set():
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as conn:
                    conn.settimeout(0.5)
                    conn.connect(self.socket_path)
                    pending, revision = b'', None
                    while not self.done.is_set():
                        data = conn.recv(65536)
                        if not data:
                            break
                        pending += data
                        if len(pending) > 1024 * 1024:
                            raise ValueError('host message too large')
                        while b'\n' in pending:
                            line, pending = pending.split(b'\n', 1)
                            state = json.loads(line)
                            with self.lock:
                                self.latest, self.received_at = state, time.monotonic()
                        with self.lock:
                            status, map_message = self.status, self.map_message
                        conn.sendall((json.dumps(status, allow_nan=False) + '\n').encode())
                        if map_message and revision != map_message['revision']:
                            conn.sendall((json.dumps(map_message, allow_nan=False) + '\n').encode())
                            revision = map_message['revision']
            except (OSError, ValueError, KeyError):
                pass
            with self.lock:
                self.latest = None
            self.done.wait(0.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True, type=Path)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    bridge = Bridge(args.config.resolve())
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.done.set()
        bridge.worker.join(timeout=1.5)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
