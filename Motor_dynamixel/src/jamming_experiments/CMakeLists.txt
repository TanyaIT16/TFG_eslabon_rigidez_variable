cmake_minimum_required(VERSION 3.0.2)
project(jamming_experiments)

## Compilar como C++11
add_compile_options(-std=c++11)

## Encontrar macros y librerías de catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_ros_library
)

## Configuración específica de catkin
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs dynamixel_ros_library
)

## Incluir directorios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declarar un ejecutable
add_executable(jamming_node src/jamming.cpp)

## Enlazar el ejecutable con las librerías necesarias
target_link_libraries(jamming_node
  ${catkin_LIBRARIES}
)