# puzzlebot_real_robot

Paquete de bringup para el **Puzzlebot fisico**. Reemplaza al paquete
`puzzlebot_gazebo` (simulacion) dentro del flujo del robot real.

## Responsabilidades

- Arrancar el **micro-ROS agent** (puente serial con el microcontrolador del Puzzlebot)
- Arrancar el **RPLiDAR A1** (driver del LiDAR fisico)
- Publicar **TF estaticas** del montaje del sensor
- Publicar **/joint_states** de las ruedas (`puzzlebot_joint_state_publisher.py`)
- Publicar **/odom** y la TF `odom -> base_footprint` (`puzzlebot_localization.py`)
- Lanzar **SLAM Toolbox** o **Nav2** con los parametros adaptados al hardware

## Estructura

```
puzzlebot_real_robot/
├── config/
│   ├── nav2_params_real.yaml      # Nav2 con use_sim_time=false, ruido real
│   └── slam_toolbox_real.yaml     # SLAM con tolerancias mayores
├── launch/
│   ├── real_robot_core.launch.xml # microROS + LiDAR + odom + TF
│   ├── slam_real.launch.xml       # core + SLAM Toolbox + RViz
│   └── nav2_real.launch.xml       # core + Nav2 bringup + RViz
├── maps/
│   ├── map_maze_real.pgm          # generado tras mapear la pista fisica
│   └── map_maze_real.yaml
├── scripts/
│   ├── puzzlebot_joint_state_publisher.py
│   └── puzzlebot_localization.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Uso

### 1) Modo SLAM (mapear la pista fisica)

```bash
ros2 launch puzzlebot_real_robot slam_real.launch.xml
# en otra terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# cuando termines:
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/puzzlebot_ros2/puzzlebot_real_robot/maps/map_maze_real
```

### 2) Modo navegacion autonoma

```bash
ros2 launch puzzlebot_real_robot nav2_real.launch.xml
# En RViz: "2D Pose Estimate" donde esta el robot
# Luego: "Nav2 Goal" para enviar destino
```

## Diferencias clave vs simulacion

| Aspecto | Simulacion | Robot fisico |
|---|---|---|
| `use_sim_time` | `true` | `false` |
| Driver del LiDAR | plugin Gazebo `libgazebo_ros_ray_sensor` | `rplidar_ros` (RPLiDAR A1 real) |
| Odometria | plugin Gazebo `libgazebo_ros_diff_drive` | `puzzlebot_localization.py` (encoders via microROS) |
| Joint states ruedas | `publish_wheel_tf=true` del plugin | `puzzlebot_joint_state_publisher.py` |
| Comunicacion robot | n/a (todo en proceso) | `micro_ros_agent` (serial USB) |
| `transform_tolerance` | 0.5 s | 1.0 s (latencia real) |
| Velocidad lineal max | 0.20 m/s | 0.15 m/s (seguridad) |
| Ruido AMCL (`alpha`) | 0.1 | 0.2 (odom real es ruidosa) |

## Dependencias del sistema

Antes de lanzar, asegurate de tener instalados:

```bash
sudo apt install \
  ros-humble-micro-ros-agent \
  ros-humble-rplidar-ros \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard
```

Conexiones tipicas:
- `/dev/ttyACM0` → micro-ROS (Puzzlebot)
- `/dev/ttyUSB0` → RPLiDAR A1

(Ajusta en `real_robot_core.launch.xml` si difieren.)
