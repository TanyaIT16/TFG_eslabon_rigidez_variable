# TFG - Eslabón de Rigidez Variable

Este repositorio almacena los códigos utilizados durante el desarrollo del Trabajo Fin de Grado Diseño y Evaluación de Eslabones Robóticos de Rigidez Variable basados Jamming Granular. Se incluyen los programas de las placas ESP32, los nodos de ROS para el control de un motor Dynamixel y los scripts de representación de datos.

## Estructura del proyecto

- **`Códigos IDE Arduino/`**: contiene los programas para las ESP32.
  - `Experimento_1/Experimento_1.ino`: mide el tiempo hasta alcanzar vacío o volver a atmósfera manejando dos relés y publicando los datos por ROS.
  - `TFG_circuito/TFG_circuito.ino`: adquiere la presión del sensor analógico y controla dos relés mediante mensajes ROS.
  - `sensorFuerza/sensorFuerza.ino`: lee una célula de carga a través del chip HX711 y publica la fuerza en ROS.

- **`Motor_dynamixel/`**: workspace de ROS con el código para controlar el motor Dynamixel.
  - `src/dynamixel_ros_library/`: librería en C++ basada en `dynamixel_sdk`.
  - `src/jamming_experiments/`: paquete ROS que implementa el nodo `jamming_node`.

- **`Representación_experimentos/`**: scripts de Python para generar las gráficas de los experimentos (`Experimento_3.py` y `Experimento_4.py`).

- **`exe/`**: incluye los archivos para ejecutar los experimentos.
  - `serial_nodes.launch`: inicia dos `serial_node` de `rosserial_python`.
  - `iniciar_experimento.sh`: script que lanza la grabación con `rosbag`, el `launch` anterior y el nodo `jamming_node`.

## Requisitos

- **ROS Noetic** (probado en Ubuntu 20.04).
- **dynamixel_sdk** y dependencias de `rosserial_python`.
- **Arduino IDE** con soporte para ESP32 y la librería **rosserial_arduino**.
- Para `sensorFuerza.ino` se necesita además la librería `HX711`.
- Para las representaciones: Python 3 con `pandas`, `numpy` y `matplotlib`.

## Compilación y carga de los programas de ESP32

1. Abrir el archivo `.ino` deseado con el **Arduino IDE**.
2. Seleccionar la placa ESP32 correspondiente y el puerto serie.
3. Compilar y cargar sobre la placa. Todos los códigos usan una velocidad de 57600 baudios para `rosserial`.

## Construcción del workspace de ROS

```bash
cd Motor_dynamixel
catkin_make
source devel/setup.bash
```

Esto compila la librería `dynamixel_ros_library` y el paquete `jamming_experiments`. El ejecutable resultante se llama **`jamming_node`**.

## Ejecución del experimento

Dentro de la carpeta `exe/` se encuentra el script que automatiza la ejecución:

```bash
./exe/iniciar_experimento.sh
```

El script inicia la grabación de todos los tópicos con `rosbag`, lanza el `launch` para los dos `serial_node` y ejecuta `rosrun jamming_experiments jamming_node` indicando el puerto, versión de protocolo y baudios del motor.

Para detenerlo basta con presionar **Ctrl+C**, lo cual finalizará todos los procesos.

## Uso de los scripts de representación

Los archivos de `Representación_experimentos` leen los csv generados durante los experimentos y muestran/guardan las gráficas comparativas.

```bash
python3 Representación_experimentos/Experimento_3.py
python3 Representación_experimentos/Experimento_4.py
```

Modifique las rutas a los datos dentro de cada script según su organización local.

