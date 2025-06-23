#!/bin/bash

# Función para detener todos los procesos limpiamente
function cleanup {
  echo "Deteniendo todos los procesos..."
  kill -INT $PID_ROSBAG
  kill -INT $PID_LAUNCH
  kill -INT $PID_NODE
  wait
  echo "Todos los procesos han sido detenidos."
}

# Capturar Ctrl+C
trap cleanup SIGINT

echo "Lanzando todo a la vez..."

# Lanzar rosbag
rosbag record -a --output-name experimento_jamming_$(date +%Y%m%d_%H%M%S).bag &
PID_ROSBAG=$!

# Lanzar roslaunch
roslaunch jamming_experiments serial_nodes.launch &
PID_LAUNCH=$!

# Lanzar rosrun
rosrun jamming_experiments jamming_node /dev/ttyUSB2 2.0 57600 &
PID_NODE=$!

# Mostrar PIDs
echo "rosbag PID: $PID_ROSBAG"
echo "roslaunch PID: $PID_LAUNCH"
echo "rosrun PID: $PID_NODE"

# Esperar hasta que se presione Ctrl+C
wait

