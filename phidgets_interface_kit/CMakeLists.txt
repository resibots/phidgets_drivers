cmake_minimum_required(VERSION 2.8.3)
project(phidgets_interface_kit)

find_package(catkin REQUIRED COMPONENTS phidgets_api roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_ir
  CATKIN_DEPENDS phidgets_api roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(phidgets_interface_kit src/main.cpp)

target_link_libraries(phidgets_ir_node ${catkin_LIBRARIES} phidgets_ir)

install(TARGETS phidgets_ir_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
