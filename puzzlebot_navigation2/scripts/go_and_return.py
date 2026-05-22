#!/usr/bin/env python3
"""
go_and_return.py — A → B → A para Puzzlebot.

Dos modos de uso:

  1. Interactivo (default si NO pasas -x/-y):
       ros2 run puzzlebot_navigation2 go_and_return.py
     Lanza el script, luego en RViz click "Nav2 Goal" en el punto B y arrastra
     la flecha. El script:
       a) toma tu pose actual como A,
       b) navega a B,
       c) espera unos segundos,
       d) regresa a A.
     Sigue escuchando: puedes mandar otro goal y repite el ciclo. Ctrl-C para salir.

  2. CLI (un solo ciclo y termina):
       ros2 run puzzlebot_navigation2 go_and_return.py -x 1.0 -y 0.5
"""

import argparse
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def current_pose(navigator: BasicNavigator, tf_buffer: Buffer,
                 timeout_sec: float = 15.0):
    """Lee TF map -> base_footprint y devuelve PoseStamped (o None si timeout)."""
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        try:
            if tf_buffer.can_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            ):
                tx = tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(),
                )
                t = tx.transform.translation
                q = tx.transform.rotation
                yaw = 2.0 * math.atan2(q.z, q.w)
                return make_pose(navigator, t.x, t.y, yaw)
        except Exception:
            pass
    return None


def go_to(navigator: BasicNavigator, label: str, pose: PoseStamped) -> bool:
    pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.get_logger().info(
        f'>> Navegando a {label}: x={pose.pose.position.x:.2f}, '
        f'y={pose.pose.position.y:.2f}'
    )
    navigator.goToPose(pose)

    while not navigator.isTaskComplete():
        fb = navigator.getFeedback()
        if fb:
            eta = Duration.from_msg(fb.estimated_time_remaining).nanoseconds / 1e9
            dist = fb.distance_remaining
            navigator.get_logger().info(
                f'   {label}: distancia={dist:.2f} m, ETA={eta:.1f} s',
                throttle_duration_sec=2.0,
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f'== {label} alcanzado')
        return True
    if result == TaskResult.CANCELED:
        navigator.get_logger().warn(f'!! {label} cancelado')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error(f'!! No se pudo alcanzar {label}')
    return False


def run_cycle(navigator: BasicNavigator, tf_buffer: Buffer,
              b_pose: PoseStamped, wait_s: float) -> bool:
    """Hace A -> B -> A. A se toma de la TF actual."""
    a_pose = current_pose(navigator, tf_buffer, timeout_sec=15.0)
    if a_pose is None:
        navigator.get_logger().error(
            'No se pudo obtener TF map->base_footprint. '
            'Verifica que fijaste la pose inicial (2D Pose Estimate) y que AMCL '
            'publica map->odom.'
        )
        return False
    navigator.get_logger().info(
        f'Punto A (TF actual): x={a_pose.pose.position.x:.2f}, '
        f'y={a_pose.pose.position.y:.2f}'
    )

    if not go_to(navigator, 'B', b_pose):
        return False

    navigator.get_logger().info(f'Espera de {wait_s:.1f} s en B...')
    time.sleep(wait_s)

    navigator.get_logger().info('Limpiando costmaps antes del regreso...')
    try:
        navigator.clearAllCostmaps()
    except Exception as e:
        navigator.get_logger().warn(f'No se pudieron limpiar costmaps: {e}')
    time.sleep(1.0)

    if not go_to(navigator, 'A (regreso)', a_pose):
        return False

    navigator.get_logger().info('Ciclo A -> B -> A completado.')
    return True


def main():
    parser = argparse.ArgumentParser(description='A -> B -> A para Puzzlebot')
    parser.add_argument('-x', type=float, default=None,
                        help='B: x (m). Si lo das, hace un solo ciclo y sale.')
    parser.add_argument('-y', type=float, default=None, help='B: y (m).')
    parser.add_argument('-Y', '--yaw', type=float, default=0.0, help='B: yaw (rad).')
    parser.add_argument('--wait', type=float, default=3.0, help='Segundos de espera en B.')
    parser.add_argument('--use-sim-time', action='store_true',
                        help='Activar SOLO en simulacion Gazebo. En robot fisico queda false.')
    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
    ])

    navigator.waitUntilNav2Active()

    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator, spin_thread=True)

    cli_b_given = args.x is not None and args.y is not None

    if cli_b_given:
        b_pose = make_pose(navigator, args.x, args.y, args.yaw)
        run_cycle(navigator, tf_buffer, b_pose, args.wait)
        rclpy.shutdown()
        return

    # Modo interactivo: escuchar /goal_pose de RViz
    pending = {'goal': None, 'busy': False}

    def on_goal(msg: PoseStamped):
        if pending['busy']:
            navigator.get_logger().warn(
                'Goal recibido pero hay un ciclo en curso, ignorando.'
            )
            return
        pending['goal'] = msg

    navigator.create_subscription(PoseStamped, '/goal_pose', on_goal, 10)
    navigator.get_logger().info(
        '>>> Modo interactivo activo. En RViz click "Nav2 Goal" sobre el punto B. '
        'El script tomara tu pose actual como A, ira a B y regresara a A. '
        'Ctrl-C para salir.'
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.2)
            if pending['goal'] is not None and not pending['busy']:
                b_pose = pending['goal']
                pending['goal'] = None
                pending['busy'] = True
                navigator.get_logger().info(
                    f'Goal de RViz recibido: x={b_pose.pose.position.x:.2f}, '
                    f'y={b_pose.pose.position.y:.2f}'
                )
                run_cycle(navigator, tf_buffer, b_pose, args.wait)
                pending['busy'] = False
                navigator.get_logger().info('>>> Esperando otro goal...')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
