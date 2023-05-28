cmake_minimum_required(VERSION 3.0.2)
project(zm_global_planner)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  std_msgs
  tf
)

generate_dynamic_reconfigure_options(
  cfg/ZMGlobalPlanner.cfg
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES zm_global_planner
   CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_core pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(zm_global_planner src/zm_global_planner.cpp
                              src/dijkstra_planner.cpp
                              src/astar_planner.cpp)
add_dependencies(zm_global_planner zm_global_planner_gencfg)
target_link_libraries(zm_global_planner ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES zm_global_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)