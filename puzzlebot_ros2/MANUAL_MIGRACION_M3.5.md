# Manual de Migración a Robot Físico — Actividad M3.5

**Equipo:** Luis Adrián Uribe Cruz, Grant Nathaniel Keegan, Diego Gerardo Sánchez Moreno, Héctor Gúmaro Guzmán Reyes
**Actividad:** M3.5 — Migración a robot real
**Repositorio:** [GitHub del equipo](https://github.com/MalvadosyAsociados3/ActividadM3.3)

---

## 1. Descripción general del proyecto

El proyecto consiste en hacer que un robot **Puzzlebot** (diferencial, con RPLiDAR A1 2D) navegue de forma autónoma a través de un laberinto. La arquitectura está basada en ROS 2 Humble y Nav2.

Esta actividad cierra el ciclo del módulo 3: tomamos el proyecto ya funcional en simulación de Gazebo y lo **extendemos** para que el mismo stack de navegación trabaje con el robot físico real, sin perder la capacidad de seguir usando la simulación para pruebas.

### Dimensiones reales del Puzzlebot

| Parámetro | Valor |
|---|---|
| Largo | 0.20 m |
| Ancho | 0.191 m |
| Diámetro de ruedas | 0.10 m |
| Separación de ruedas | 0.19 m |
| Footprint (rectángulo) | `[[0.10, 0.0955], [0.10, -0.0955], [-0.10, -0.0955], [-0.10, 0.0955]]` |
| LiDAR | RPLiDAR A1 (2D, 360°, 0.15–12 m) |
| Pasillos del laberinto | 60 cm |

---

## 2. Estructura final del proyecto

```
puzzlebot_ros2/
├── puzzlebot_description/        # URDF/Xacro y meshes (compartido sim + real)
├── puzzlebot_gazebo/             # SIMULACIÓN: Gazebo, mundo, spawn, bridge
├── puzzlebot_navigation2/        # SIMULACIÓN: Nav2 + SLAM + maps + RViz
└── puzzlebot_real_robot/         # NUEVO — HARDWARE REAL
    ├── config/
    │   ├── nav2_params_real.yaml     # use_sim_time=false, topics reales
    │   └── slam_toolbox_real.yaml    # tolerancias ajustadas
    ├── launch/
    │   ├── real_robot_core.launch.xml   # microROS + LiDAR + odom + TF
    │   ├── slam_real.launch.xml         # core + SLAM Toolbox
    │   └── nav2_real.launch.xml         # core + Nav2 bringup
    ├── maps/
    │   ├── map_maze_real.pgm            # generado en la pista física
    │   └── map_maze_real.yaml
    ├── scripts/
    │   ├── puzzlebot_joint_state_publisher.py
    │   └── puzzlebot_localization.py
    ├── CMakeLists.txt
    ├── package.xml
    └── README.md
```

**Principio rector:** el paquete `puzzlebot_real_robot` **reemplaza** a `puzzlebot_gazebo` en el flujo de ejecución del robot físico. Ningún launch del robot real depende del paquete de simulación.

---

## 3. Paquetes reutilizados del proyecto anterior

| Paquete | Reutilizado en robot real | Justificación |
|---|---|---|
| `puzzlebot_description` | ✅ Sí | URDF y meshes son los mismos. Los tags `<gazebo>` del URDF son ignorados por `robot_state_publisher` en hardware real. |
| `puzzlebot_navigation2` | ✅ Parcialmente | Reutilizamos los perfiles de RViz (`slam.rviz`, `nav2.rviz`). Los `nav2_params.yaml` y `slam_toolbox.yaml` NO se reutilizan; el robot real usa los suyos. |
| `puzzlebot_gazebo` | ❌ No | Exclusivo de simulación. Sólo se usa para `slam.launch.py`/`nav2.launch.py` en Gazebo. |

---

## 4. Paquete nuevo: `puzzlebot_real_robot`

### 4.1 Responsabilidades

- **micro-ROS agent**: puente serial con el microcontrolador del Puzzlebot
- **RPLiDAR A1 driver**: publica `/scan` real
- **`robot_state_publisher`**: publica TF a partir del URDF
- **`puzzlebot_joint_state_publisher.py`**: publica `/joint_states` integrando `/wl`, `/wr` (encoders via microROS) → así las ruedas se ven girar en RViz
- **`puzzlebot_localization.py`**: integra cinemática diferencial y publica `/odom` + TF `odom → base_footprint`
- **TF estática**: ajuste de montaje del LiDAR (`lidar_link → laser`)
- **Lanzamiento integrado** de SLAM o Nav2

### 4.2 Launch files

#### `real_robot_core.launch.xml`
Es el "boot" del robot. Lanza todo el hardware:

```xml
<launch>
  <node pkg="micro_ros_agent" exec="micro_ros_agent" .../>
  <node pkg="rplidar_ros" exec="rplidar_composition" .../>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" .../>
  <node pkg="puzzlebot_real_robot" exec="puzzlebot_joint_state_publisher.py" .../>
  <node pkg="puzzlebot_real_robot" exec="puzzlebot_localization.py" .../>
  <node pkg="tf2_ros" exec="static_transform_publisher" .../>
</launch>
```

Es **incluido** por los otros dos launch (slam y nav2), no se lanza por separado. Esto cumple con la expectativa del PDF de no tener que ejecutar múltiples launch files manualmente.

#### `slam_real.launch.xml`
Modo mapeo: incluye `real_robot_core` + arranca `async_slam_toolbox_node` con `slam_toolbox_real.yaml` + RViz con el perfil `slam.rviz` (reutilizado de navigation2).

#### `nav2_real.launch.xml`
Modo navegación: incluye `real_robot_core` + `nav2_bringup/bringup_launch.py` con `nav2_params_real.yaml` y el mapa real + RViz con `nav2.rviz`.

### 4.3 Scripts

#### `puzzlebot_joint_state_publisher.py`
Suscribe `/wl`, `/wr` (Float32, velocidades angulares de cada rueda publicadas por el firmware via micro-ROS) y publica `/joint_states` integrando las posiciones. Esto reemplaza el `publish_wheel_tf=true` del plugin de Gazebo.

#### `puzzlebot_localization.py`
Implementa la cinemática diferencial:
```
v     = r * (wr + wl) / 2
omega = r * (wr - wl) / d
```
Integra (x, y, θ) con Euler a 30 Hz y publica:
- `/odom` (`nav_msgs/Odometry`)
- TF `odom → base_footprint`

Reemplaza el plugin `libgazebo_ros_diff_drive`.

> **Importante**: si el firmware del Puzzlebot ya publica `/odom` y la TF, este script **NO** debe correr, para evitar duplicar la fuente de odometría (uno de los errores típicos al migrar).

---

## 5. Cambios respecto al proyecto en simulación

### 5.1 URDF/Xacro

| Cambio | Antes (sim escalada al 70%) | Ahora (real 100%) |
|---|---|---|
| `mesh_scale` | 0.7 | **1.0** |
| `base_length` | 0.126 m | **0.20 m** |
| `base_width` | 0.105 m | **0.191 m** |
| `wheel_radius` | 0.035 m | **0.05 m** |
| `wheel_separation` | 0.133 m | **0.19 m** |
| `base_link z` (sobre footprint) | 0.0368 m | **0.0525 m** |
| Posición LiDAR z | 0.08 m | 0.08 m (sin cambio, ya estaba bien) |

Los tags `<gazebo>` y `<plugin>` se quedan en el URDF pero el robot real los ignora (sólo afectan a Gazebo).

### 5.2 Spawn del robot

- **Antes**: (0.0, 0.0, yaw=0) — centro del mapa
- **Ahora**: **(1.35, 0.0, yaw=π)** — puerta de entrada del laberinto

Este era uno de los errores señalados en la retroalimentación M3.4: el robot debía partir desde una puerta y llegar a la otra, no del centro.

### 5.3 `nav2_params.yaml` (simulación) — correcciones aplicadas

Con base en la retroalimentación del M3.4:

| Punto retroalimentado | Corrección aplicada |
|---|---|
| Robot escalado al 70% incorrecto | Robot a tamaño real (0.20 × 0.191 m) |
| `robot_radius: 0.08` subestima al robot | Reemplazado por `footprint` polígono rectangular |
| `voxel_layer` innecesario en LiDAR 2D | Eliminado; solo `obstacle_layer + inflation_layer` |
| `static_layer` definido pero no en plugins | Movido a la lista de plugins del global_costmap |
| Frame inconsistente (`base_link` vs `base_footprint`) | Todo el stack usa `base_footprint` |
| Planner: comentario decía A* pero `use_astar: false` | `use_astar: true` (alineado con la intención) |
| `allow_unknown: true` con mapa cerrado | `allow_unknown: false` |
| `inflation_radius: 0.15` (basado en pasillos 30 cm) | `inflation_radius: 0.20` (pasillos 60 cm) |

### 5.4 `package.xml` de `puzzlebot_navigation2` — limpieza

Quitamos las dependencias redundantes que ya provee `nav2_bringup`:

```diff
- <exec_depend>nav2_amcl</exec_depend>
- <exec_depend>nav2_map_server</exec_depend>
- <exec_depend>nav2_planner</exec_depend>
- <exec_depend>nav2_controller</exec_depend>
- <exec_depend>nav2_bt_navigator</exec_depend>
- <exec_depend>nav2_lifecycle_manager</exec_depend>
```

### 5.5 `nav2_params_real.yaml` vs `nav2_params.yaml`

| Parámetro | Simulación | Robot real | Razón |
|---|---|---|---|
| `use_sim_time` | true | **false** | Sin reloj de Gazebo |
| `alpha1..5` (AMCL) | 0.1 | **0.2** | Odom real es ruidosa |
| `transform_tolerance` (AMCL) | 0.5 s | **1.0 s** | Latencia WiFi/microROS |
| `desired_linear_vel` (RPP) | 0.20 m/s | **0.15 m/s** | Seguridad física |
| `update_frequency` (local) | 10 Hz | **5 Hz** | CPU Jetson Nano |
| `controller_frequency` | 20 Hz | **15 Hz** | idem |
| `footprint_padding` | 0.02 | **0.03** | Imprecisión real |
| `transform_timeout` (recoveries) | 0.1 s | **0.2 s** | Latencia real |

### 5.6 `slam_toolbox_real.yaml` vs `slam_toolbox.yaml`

- `use_sim_time: false`
- `max_laser_range: 6.0` (pista no es más grande; reduce ruido del LiDAR a distancia larga)
- `transform_timeout: 0.5` (vs 0.2 en sim)
- `transform_publish_period: 0.05` (vs 0.02 en sim, menos carga)

---

## 6. Cómo ejecutar SLAM en el robot físico

### Prerrequisitos
```bash
sudo apt install \
  ros-humble-micro-ros-agent \
  ros-humble-rplidar-ros \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard
```

Verifica conexiones:
```bash
ls /dev/ttyACM*   # Puzzlebot (microROS)
ls /dev/ttyUSB*   # RPLiDAR
```

### Pasos
```bash
# Terminal 1: SLAM
cd ~/ros2_ws
source install/setup.bash
ros2 launch puzzlebot_real_robot slam_real.launch.xml

# Terminal 2: teleop para recorrer el laberinto
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Cuando termines de recorrer TODO el laberinto:
# Terminal 3: guardar el mapa
source ~/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/puzzlebot_ros2/puzzlebot_real_robot/maps/map_maze_real
```

---

## 7. Cómo ejecutar navegación en el robot físico

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch puzzlebot_real_robot nav2_real.launch.xml
```

En RViz:
1. **2D Pose Estimate** → marca la posición real del robot en el mapa
2. Verifica que el LaserScan se pegue a las paredes del mapa
3. **Nav2 Goal** → marca el destino (la otra puerta del laberinto)

---

## 8. Secciones "simulation only"

Quedan claramente aisladas dentro del paquete `puzzlebot_gazebo` y del URDF:

| Elemento | Ubicación | Sólo simulación |
|---|---|---|
| `<gazebo>` materials | `puzzlebot_description/urdf/gazebo_plugins.xacro` | ✅ |
| Plugin `libgazebo_ros_diff_drive` | mismo archivo | ✅ |
| Plugin `libgazebo_ros_ray_sensor` | mismo archivo | ✅ |
| Launch de Gazebo (gzserver + gzclient) | `puzzlebot_gazebo/launch/puzzlebot_gazebo.launch.py` | ✅ |
| Bridge YAML | `puzzlebot_gazebo/config/gazebo_bridge.yaml` | ✅ |
| Mundo SDF | `puzzlebot_gazebo/worlds/maze_world.world` | ✅ |
| `gazebo_ros2_control` (si se usa) | URDF + plugins | ✅ |

Estos elementos no se eliminan del proyecto, simplemente nunca son cargados por los launch files del robot físico.

---

## 9. Problemas encontrados y cómo se resolvieron

### 9.1 Conflicto de workspace en `AMENT_PREFIX_PATH`
**Problema**: `ros2 pkg prefix puzzlebot_gazebo` apuntaba a un workspace antiguo (`~/robotec_sim_ws`) en vez de `~/ros2_ws`, porque al construir `ros2_ws` se había sourceado el otro workspace, dejándolo como underlay automático.

**Solución**: filtrar `AMENT_PREFIX_PATH` al inicio de cada terminal:
```bash
export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -v robotec_sim_ws | paste -sd:)
```
O bien, recompilar `ros2_ws` desde una terminal sin el `.bashrc` para que no encadene al otro workspace.

### 9.2 AMCL divergente al cambiar de tamaño de robot
**Problema**: al pasar del Puzzlebot escalado al 70% al tamaño real (100%), AMCL no convergía bien con el mapa anterior.

**Solución**: regenerar el mapa con el robot a tamaño real, antes de cualquier prueba de navegación.

### 9.3 Robot no llegaba al destino correcto (M3.4 review)
**Problema**: el spawn estaba en (0, 0) cuando debía ser en la puerta del laberinto (1.35, 0.0).

**Solución**: actualizar `puzzlebot_gazebo.launch.py` y los defaults de `set_initial_pose.py` con la nueva pose.

### 9.4 Costmap desfasado en RViz durante el movimiento
**Problema**: el costmap se veía corrido cuando el robot se movía.

**Solución**: alinear todos los frames a `base_footprint` (antes algunos nodos usaban `base_link`) y aumentar `transform_tolerance` en AMCL.

### 9.5 Loop closures falsos en SLAM
**Problema**: SLAM "teletransportaba" el robot al cerrar loops falsos en pasillos parecidos.

**Solución**: endurecer thresholds (`loop_match_minimum_response_*: 0.55/0.65`, `correlation_search_space_dimension: 0.3`, `*_variance_penalty` aumentados).

### 9.6 Frontera entre simulación y real
**Problema**: tags de Gazebo en el mismo URDF que se usa para hardware real.

**Solución**: aceptado porque `robot_state_publisher` los ignora. Si fuera necesario para limpieza, se podría separar en `puzzlebot.urdf.xacro` (real) y `puzzlebot_sim.urdf.xacro` (con plugins), pero no fue requerido para el funcionamiento.

---

## 10. Checklist final (PDF de M3.5)

- [x] Existencia de un paquete específico para el robot físico (`puzzlebot_real_robot`)
- [x] Existencia de launch file para SLAM en hardware real (`slam_real.launch.xml`)
- [x] Existencia de launch file para navegación en hardware real (`nav2_real.launch.xml`)
- [x] Integración de los launch files del hardware dentro de su propia arquitectura (un solo `real_robot_core.launch.xml` agrupa microROS + LiDAR + odom + TF; los otros lo incluyen)
- [x] Capacidad de volver a correr SLAM sobre la pista física
- [x] Documentación clara del proceso de migración (este manual)
- [x] Carpeta `scripts/` con scripts realmente funcionales y propósito claro

---

## 11. Conclusión

La migración se sustenta en **dos principios** que el PDF M3.5 enfatiza:

1. **Separación clara entre simulación y hardware real**: lo que antes hacía `puzzlebot_gazebo` (mundo, spawn, bridge, plugins) ahora lo cubre `puzzlebot_real_robot` con piezas distintas (microROS, RPLiDAR driver, scripts de odom y joints). Nada del flujo del robot real depende de Gazebo.

2. **Reutilización inteligente**: el URDF, los meshes y los perfiles de RViz son comunes a ambos modos; sólo cambia el bringup. Esto reduce código duplicado y facilita mantener consistencia.

El resultado es un proyecto donde lanzar **`ros2 launch puzzlebot_real_robot nav2_real.launch.xml`** levanta todo lo necesario (hardware + Nav2 + RViz) con un solo comando, sin pasos manuales adicionales.

---

## Anexo: enlaces

- **Repositorio:** https://github.com/MalvadosyAsociados3/ActividadM3.3
- **Video de simulación M3.4 (referencia):** [link al video del equipo]
- **Video opcional de robot físico:** [agregar si se hace la demo]
