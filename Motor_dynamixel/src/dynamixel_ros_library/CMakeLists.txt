cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_ros_library)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Package dependencies
set(${PROJECT_NAME}_DEPENDS
  roscpp
  std_msgs
  message_generation
  message_runtime
  dynamixel_sdk
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

## Export (include dirs, libraries, etc)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DynamixelSDK_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/dynamixel_ros_library.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${DynamixelSDK_LIBRARIES}
)

set(libraries "dynamixel_ros_library")

## Example Nodes

# Test communication node
add_executable(testCommunication src/testCommunication.cpp)
add_dependencies(testCommunication ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testCommunication
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Blinking LED node
add_executable(blinkingLED src/blinkingLED.cpp)
add_dependencies(blinkingLED ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(blinkingLED
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Test position control node
add_executable(testPositionControl src/testPositionControl.cpp)
add_dependencies(testPositionControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testPositionControl
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Test PWM control node
add_executable(testPWMControl src/testPWMControl.cpp)
add_dependencies(testPWMControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(testPWMControl
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Dynamixel Parameters monitor node
add_executable(dmxlParamsMonitor src/dmxlParamsMonitor.cpp)
add_dependencies(dmxlParamsMonitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dmxlParamsMonitor
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(pinzaTest src/realTest.cpp)
add_dependencies(pinzaTest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pinzaTest
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(rolling_3_fingers src/rolling_3_fingers.cpp)
add_dependencies(rolling_3_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_3_fingers
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(fsm_3_fingers src/fsm_3_fingers.cpp)
add_dependencies(fsm_3_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fsm_3_fingers
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(rolling_4_fingers src/rolling_4_fingers.cpp)
add_dependencies(rolling_4_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rolling_4_fingers
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(fsm_4_fingers src/fsm_4_fingers.cpp)
add_dependencies(fsm_4_fingers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fsm_4_fingers
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Jamming control node
#add_executable(jamming src/jamming.cpp)
#add_dependencies(jamming ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(jamming
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)