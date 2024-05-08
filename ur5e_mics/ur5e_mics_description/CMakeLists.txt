cmake_minimum_required(VERSION 3.0.2)
project(ur5e_mics_description)

find_package(catkin REQUIRED COMPONENTS
  ur_description
  # robotiq_2f_85_gripper_visualization
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)