cmake_minimum_required(VERSION 3.5)
project(ocs2_legged_robot_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ocs2_thirdparty REQUIRED)
find_package(OpenMP REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(AMENT_DEPENDENCIES
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  urdfdom
  kdl_parser
  rclcpp
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_ipm
  ocs2_mpc
  ocs2_sqp
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_legged_robot
)

foreach(Dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")

find_package(pinocchio REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Main library
add_library(${PROJECT_NAME}
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/LeggedRobotVisualizer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# DDP-MPC node for legged robot
add_executable(legged_robot_ddp_mpc
  src/LeggedRobotDdpMpcNode.cpp
)
target_include_directories(legged_robot_ddp_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(legged_robot_ddp_mpc
${AMENT_DEPENDENCIES})

add_dependencies(legged_robot_ddp_mpc
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(legged_robot_ddp_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_ddp_mpc PRIVATE ${OCS2_CXX_FLAGS})

## SQP-MPC node for legged robot
add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
)

ament_target_dependencies(legged_robot_sqp_mpc
${AMENT_DEPENDENCIES})

add_dependencies(legged_robot_sqp_mpc
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_sqp_mpc
  ${PROJECT_NAME}
)
ament_target_dependencies(legged_robot_sqp_mpc ${PACKAGE_DEPENDENCIES})
target_compile_options(legged_robot_sqp_mpc  PRIVATE ${OCS2_CXX_FLAGS})

## IPM-MPC node for legged robot
add_executable(legged_robot_ipm_mpc
  src/LeggedRobotIpmMpcNode.cpp
)

ament_target_dependencies(legged_robot_ipm_mpc
${AMENT_DEPENDENCIES})

add_dependencies(legged_robot_ipm_mpc
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_ipm_mpc
  ${PROJECT_NAME}
)
ament_target_dependencies(legged_robot_ipm_mpc ${PACKAGE_DEPENDENCIES})
target_compile_options(legged_robot_ipm_mpc  PRIVATE ${OCS2_CXX_FLAGS})

# # Dummy node
add_executable(legged_robot_dummy
  src/LeggedRobotDummyNode.cpp
)
target_include_directories(legged_robot_dummy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(legged_robot_dummy
${AMENT_DEPENDENCIES})

add_dependencies(legged_robot_dummy
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(legged_robot_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_dummy PRIVATE ${OCS2_CXX_FLAGS})

# # Keyboard command node for target trajectories
add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
target_include_directories(legged_robot_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_dependencies(legged_robot_target
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(legged_robot_target
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(legged_robot_gait_command
  src/LeggedRobotGaitCommandNode.cpp
)
target_include_directories(legged_robot_gait_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_dependencies(legged_robot_gait_command
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(legged_robot_gait_command
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_gait_command PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
   add_clang_tooling(
     TARGETS
       ${PROJECT_NAME}
       legged_robot_ddp_mpc
       legged_robot_sqp_mpc
       legged_robot_ipm_mpc
       legged_robot_dummy
       legged_robot_target
       legged_robot_gait_command
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS
    legged_robot_ddp_mpc
    legged_robot_sqp_mpc
    legged_robot_ipm_mpc
    legged_robot_dummy
    legged_robot_target
    legged_robot_gait_command
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(ocs2_core)
ament_export_dependencies(ocs2_mpc)
ament_export_dependencies(ocs2_robotic_tools)
ament_export_dependencies(ocs2_oc)
ament_export_dependencies(ocs2_thirdparty)
ament_export_dependencies(ocs2_ddp)
ament_export_dependencies(ocs2_msgs)
ament_export_dependencies(ocs2_sqp)
ament_export_dependencies(ocs2_robotic_assets)
ament_export_dependencies(ocs2_pinocchio_interface)
ament_export_dependencies(ocs2_centroidal_model)
ament_export_dependencies(ocs2_ros_interfaces)
ament_export_dependencies(tf2)
ament_export_dependencies(urdf)
ament_export_dependencies(urdfdom)
ament_export_dependencies(kdl_parser)
ament_export_dependencies(robot_state_publisher)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(ocs2_ipm)
ament_package()