cmake_minimum_required(VERSION 2.8.3)
project(phidgets_interface_kit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  phidgets_api
)

add_service_files (
  FILES
  getState.srv
  getStates.srv
  setState.srv
  setStates.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_ir
  CATKIN_DEPENDS roscpp message_runtime phidgets_api
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(phidgets_interface_kit src/main.cpp src/interface_kit_node.cpp)

target_link_libraries(phidgets_interface_kit ${catkin_LIBRARIES} phidgets_api)

add_dependencies(phidgets_interface_kit phidgets_interface_kit_gencpp phidgets_interface_kit_messages_cpp)
