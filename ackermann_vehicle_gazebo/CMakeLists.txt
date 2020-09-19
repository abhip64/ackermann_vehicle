# CMakeLists.txt
#
# Copyright (c) 2015 Wunderkammer Laboratory
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.3)
project(ackermann_vehicle_gazebo)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  std_msgs
  ackermann_msgs
  tf
  mav_trajectory_generation
  apriltag_ros
)

catkin_package()

catkin_install_python(PROGRAMS scripts/ackermann_controller
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ackermann_traj src/ackermann_traj.cpp)

add_dependencies(ackermann_traj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(ackermann_traj
   ${catkin_LIBRARIES}
 )

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
