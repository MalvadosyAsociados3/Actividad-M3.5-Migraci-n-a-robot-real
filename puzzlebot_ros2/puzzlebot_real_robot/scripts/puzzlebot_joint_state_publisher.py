#!/usr/bin/env python3
"""
puzzlebot_joint_state_publisher.py

Publica /joint_states de las ruedas a partir de los datos crudos de encoders
que el firmware del Puzzlebot envia por micro-ROS.

Suscripciones (esperadas del firmware via microROS):
  - /wl   (std_msgs/Float32)  velocidad angular rueda izquierda [rad/s]
  - /wr   (std_msgs/Float32)  velocidad angular rueda derecha  [rad/s]

Publica:
  - /joint_states (sensor_msgs/JointState) con
    name=[wheel_l_joint, wheel_r_joint] y posiciones integradas en tiempo.

Esto permite que robot_state_publisher dibuje las ruedas girando en RViz.

Nota: en el robot fisico no hay plugin de Gazebo que publique wheel_tf,
por eso este nodo cumple esa funcion via robot_state_publisher.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class PuzzlebotJointStatePublisher(Node):
    def __init__(self):
        super().__init__('puzzlebot_joint_state_publisher')

        # Estado: posiciones angulares integradas
        self.theta_l = 0.0
        self.theta_r = 0.0
        self.wl = 0.0
        self.wr = 0.0
        self.last_time = self.get_clock().now()

        # Subscripciones a las velocidades de las ruedas (firmware microROS)
        self.create_subscription(Float32, '/wl', self._cb_wl, 10)
        self.create_subscription(Float32, '/wr', self._cb_wr, 10)

        # Publicador de joint_states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer a 30 Hz para integrar y publicar
        self.create_timer(1.0 / 30.0, self._tick)

        self.get_logger().info('puzzlebot_joint_state_publisher listo (escuchando /wl, /wr).')

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

        # Integrar la posicion angular
        self.theta_l += self.wl * dt
        self.theta_r += self.wr * dt

        # Mantenerlo en [-pi, pi] para que JointState no crezca sin limite
        self.theta_l = math.atan2(math.sin(self.theta_l), math.cos(self.theta_l))
        self.theta_r = math.atan2(math.sin(self.theta_r), math.cos(self.theta_r))

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ['wheel_l_joint', 'wheel_r_joint']
        msg.position = [self.theta_l, self.theta_r]
        msg.velocity = [self.wl, self.wr]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PuzzlebotJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
