# maps/

Mapas del entorno fisico generados con SLAM Toolbox sobre la pista real.

## Archivos esperados

- `map_maze_real.pgm` — imagen de ocupacion (grayscale)
- `map_maze_real.yaml` — metadatos (resolucion, origen, umbrales)

Estos archivos se generan ejecutando:

```bash
ros2 launch puzzlebot_real_robot slam_real.launch.xml
# manejar el robot recorriendo TODA la pista
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/puzzlebot_ros2/puzzlebot_real_robot/maps/map_maze_real
```

## Diferencias con el mapa de simulacion

El mapa real **no** sera identico al de simulacion porque:

- Las dimensiones reales de la pista pueden tener pequenas variaciones
- El LiDAR fisico (RPLiDAR A1) tiene ruido distinto al simulado
- Superficies oscuras o reflejantes pueden distorsionar lecturas
- La odometria de los encoders acumula error
- Las paredes pueden no estar perfectamente perpendiculares

Por eso es necesario **regenerar el mapa** sobre la pista real antes de
ejecutar `nav2_real.launch.xml`.
