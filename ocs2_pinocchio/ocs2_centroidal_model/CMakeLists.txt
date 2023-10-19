cmake_minimum_required(VERSION 3.5)
project(ocs2_centroidal_model)
find_package(ament_cmake REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_DEPENDENCIES
  ocs2_core
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  pinocchio
  Eigen3
)

# find_package(${AMENT_DEPENDENCIES})
foreach(Dependency IN ITEMS ${AMENT_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log 
  log_setup
)

find_package(Threads REQUIRED)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ocs2_core_INCLUDE_DIRS}
  ${ocs2_robotic_tools_INCLUDE_DIRS}
  ${ocs2_robotic_assets_INCLUDE_DIRS}
  ${ocs2_pinocchio_interface_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/PinocchioCentroidalDynamics.cpp
  src/PinocchioCentroidalDynamicsAD.cpp
  src/CentroidalModelRbdConversions.cpp
  src/CentroidalModelPinocchioMapping.cpp
  src/CentroidalModelInfo.cpp
  src/FactoryFunctions.cpp
  src/ModelHelperFunctions.cpp
)
ament_target_dependencies(${PROJECT_NAME}
${AMENT_DEPENDENCIES})

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

#############
## Testing ##
#############

# catkin_add_gtest(${PROJECT_NAME}_test
#   # test/testAccessMethods.cpp
#   test/testAnymalCentroidalModel.cpp
# )
# target_include_directories(${PROJECT_NAME}_test PRIVATE
#   test/include
# )
# target_link_libraries(${PROJECT_NAME}_test
#   gtest_main
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${pinocchio_LIBRARIES}
# )
# target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})

ament_export_dependencies(${AMENT_DEPENDENCIES} pinocchio Boost)
ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_package()

