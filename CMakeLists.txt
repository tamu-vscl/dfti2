cmake_minimum_required(VERSION 2.8.3)
project(dfti2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  dftiData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dfti2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dfti src/dfti.cpp)
target_link_libraries(dfti ${catkin_LIBRARIES})
add_dependencies(dfti ${catkin_EXPORTED_TARGETS})

add_executable(data_reformater src/data_reformater.cpp)
target_link_libraries(data_reformater ${catkin_LIBRARIES})
add_dependencies(data_reformater ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS src/arduino_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
