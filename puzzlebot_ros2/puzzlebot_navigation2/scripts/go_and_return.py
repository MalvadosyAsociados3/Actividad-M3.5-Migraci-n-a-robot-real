#!/usr/bin/env python3
"""
go_and_return.py  --  Demuestra el requisito M3.4 "ir al punto B y regresar al A".

Flujo:
  1. Guarda la pose actual del robot como punto A.
  2. Navega al goal B (definido por CLI o defaults).
  3. Espera unos segundos en B.
  4. Regresa al punto A.


"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def go_to(navigator: BasicNavigator, label: str, pose: PoseStamped) -> bool:
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
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn(f'!! {label} cancelado')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error(f'!! No se pudo alcanzar {label}')
    return False


def main():
    parser = argparse.ArgumentParser(description='Ir al punto B y regresar al A')
    parser.add_argument('-x', type=float, default=1.0, help='Goal B: x (m)')
    parser.add_argument('-y', type=float, default=0.5, help='Goal B: y (m)')
    parser.add_argument('-Y', '--yaw', type=float, default=0.0, help='Goal B: yaw (rad)')
    parser.add_argument('--ax', type=float, default=None, help='A: x (default = pose actual)')
    parser.add_argument('--ay', type=float, default=None, help='A: y (default = pose actual)')
    parser.add_argument('--ayaw', type=float, default=None, help='A: yaw (default = pose actual)')
    parser.add_argument('--wait', type=float, default=3.0, help='Segundos de espera en B')
    parser.add_argument('--use-sim-time', action='store_true',
                        help='Activar SOLO en simulacion Gazebo. En robot fisico debe quedar false.')
    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()

    # use_sim_time debe coincidir con el resto del stack: true en Gazebo,
    # false en robot fisico (no existe /clock simulado). Mezclar tiempos
    # rompe el TransformListener y AMCL.
    from rclpy.parameter import Parameter
    navigator.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time),
    ])

    navigator.waitUntilNav2Active()

    # Determinar punto A
    if args.ax is not None and args.ay is not None and args.ayaw is not None:
        a_pose = make_pose(navigator, args.ax, args.ay, args.ayaw)
        navigator.get_logger().info(
            f'Punto A (manual): x={args.ax:.2f}, y={args.ay:.2f}, yaw={args.ayaw:.2f}'
        )
    else:
        # Usar la pose actual del robot (TF map->base_footprint) como punto A.
        # IMPORTANTE: dejar tiempo al TransformListener para que reciba los
        # mensajes /tf antes de intentar el lookup.
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, navigator, spin_thread=True)

        navigator.get_logger().info('Esperando TF map->base_footprint...')
        timeout_sec = 15.0
        deadline = time.time() + timeout_sec
        transform = None

        while time.time() < deadline:
            # Spin un poco para dejar que el listener procese /tf
            rclpy.spin_once(navigator, timeout_sec=0.1)
            try:
                if tf_buffer.can_transform(
                    'map', 'base_footprint', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                ):
                    transform = tf_buffer.lookup_transform(
                        'map', 'base_footprint', rclpy.time.Time()
                    )
                    break
            except Exception:
                pass

        if transform is None:
            navigator.get_logger().error(
                'No se pudo obtener TF map->base_footprint tras 15 s. '
                'Verifica que fijaste la pose inicial (2D Pose Estimate) '
                'y que AMCL publica map->odom. '
                'Como alternativa, pasa --ax --ay --ayaw manualmente.'
            )
            # no apagamos Nav2 para mantener activos map_server/AMCL
            rclpy.shutdown()
            sys.exit(1)

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        a_pose = make_pose(navigator, t.x, t.y, yaw)
        navigator.get_logger().info(
            f'Punto A (TF actual): x={t.x:.2f}, y={t.y:.2f}, yaw={yaw:.2f}'
        )

    b_pose = make_pose(navigator, args.x, args.y, args.yaw)
    navigator.get_logger().info(
        f'Punto B:             x={args.x:.2f}, y={args.y:.2f}, yaw={args.yaw:.2f}'
    )

    # A -> B
    if not go_to(navigator, 'B', b_pose):
        # no apagamos Nav2 para mantener activos map_server/AMCL
        rclpy.shutdown()
        sys.exit(1)

    navigator.get_logger().info(f'Espera de {args.wait:.1f} s en B...')
    time.sleep(args.wait)

    # Limpiar costmaps antes del regreso para que el planner pueda replanear.
    # Sin esto, residuos del costmap local pueden hacer que el robot parezca
    # estar "dentro de un obstaculo inflado" y el planner falle.
    navigator.get_logger().info('Limpiando costmaps antes del regreso...')
    try:
        navigator.clearAllCostmaps()
    except Exception as e:
        navigator.get_logger().warn(f'No se pudieron limpiar costmaps: {e}')
    time.sleep(1.0)

    # B -> A
    # Re-timestamp antes de enviar
    a_pose.header.stamp = navigator.get_clock().now().to_msg()
    if not go_to(navigator, 'A (regreso)', a_pose):
        # no apagamos Nav2 para mantener activos map_server/AMCL
        rclpy.shutdown()
        sys.exit(1)

    navigator.get_logger().info('Ciclo A -> B -> A completado.')
    # no apagamos Nav2 para mantener activos map_server/AMCL
    rclpy.shutdown()


if __name__ == '__main__':
    main()
