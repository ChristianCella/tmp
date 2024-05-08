cmake_minimum_required(VERSION 3.0.2)
project(ur5e_mics_bringup)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  ur_dashboard_msgs
  ur_msgs
)
catkin_package(
  INCLUDE_DIRS #include 
  LIBRARIES #${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  std_srvs ur_dashboard_msgs ur_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# make node executable 
add_executable(${PROJECT_NAME}_node src/node.cpp
  )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

