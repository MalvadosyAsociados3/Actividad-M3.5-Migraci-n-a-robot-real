#!/usr/bin/env python3
"""
puzzlebot_localization.py

Localizacion por dead reckoning (odometria de encoders) para el Puzzlebot fisico.

Modelo cinematico de robot diferencial:
  v     = r * (wr + wl) / 2
  omega = r * (wr - wl) / d

donde:
  r  = radio de rueda
  d  = separacion entre ruedas
  wl = velocidad angular rueda izquierda  (rad/s)
  wr = velocidad angular rueda derecha    (rad/s)

Integra la pose (x, y, theta) y publica:
  - /odom  (nav_msgs/Odometry)
  - TF odom -> base_footprint

Esto reemplaza al plugin libgazebo_ros_diff_drive que se usaba en simulacion.

Si el firmware ya publica /odom y la TF, NO incluyas este nodo (evita doble
fuente de odometria, que es uno de los errores tipicos al migrar a real).
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PuzzlebotLocalization(Node):
    def __init__(self):
        super().__init__('puzzlebot_localization')

        # Parametros fisicos (deben coincidir con URDF)
        self.declare_parameter('wheel_radius',     0.05)
        self.declare_parameter('wheel_separation', 0.19)
        self.declare_parameter('publish_tf',       True)
        self.declare_parameter('odom_frame',       'odom')
        self.declare_parameter('base_frame',       'base_footprint')
        self.declare_parameter('rate_hz',          30.0)

        self.r   = float(self.get_parameter('wheel_radius').value)
        self.d   = float(self.get_parameter('wheel_separation').value)
        self.pub_tf      = bool(self.get_parameter('publish_tf').value)
        self.odom_frame  = str(self.get_parameter('odom_frame').value)
        self.base_frame  = str(self.get_parameter('base_frame').value)
        rate_hz          = float(self.get_parameter('rate_hz').value)

        # Estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wl = 0.0
        self.wr = 0.0
        self.last_time = self.get_clock().now()

        # I/O
        self.create_subscription(Float32, '/wl', self._cb_wl, 10)
        self.create_subscription(Float32, '/wr', self._cb_wr, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.pub_tf else None

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            f'puzzlebot_localization listo (r={self.r}, d={self.d}, '
            f'publish_tf={self.pub_tf}).'
        )

    def _cb_wl(self, msg: Float32):
        self.wl = msg.data

    def _cb_wr(self, msg: Float32):
        self.wr = msg.data

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        # Cinematica diferencial
        v     = self.r * (self.wr + self.wl) / 2.0
        omega = self.r * (self.wr - self.wl) / self.d

        # Integracion (modelo Euler suficiente a 30 Hz para vel bajas)
        self.theta += omega * dt
        # Normalizar yaw
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        stamp = now.to_msg()
        q = yaw_to_quaternion(self.theta)

        # ── /odom ──
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega
        # Covarianzas conservadoras (encoders ruidosos)
        odom.pose.covariance[0]  = 0.05
        odom.pose.covariance[7]  = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0]  = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        # ── TF odom -> base_footprint ──
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = PuzzlebotLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
