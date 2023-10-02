cmake_minimum_required(VERSION 3.5)
project(ocs2_robotic_tools)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${ocs2_core_INCLUDE_DIRS}
  ${ocs2_oc_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/common/RotationTransforms.cpp
  src/common/LoopshapingRobotInterface.cpp
)
ament_target_dependencies(${PROJECT_NAME}
${ocs2_core_Libraries}
${ocs2_oc_Libraries}
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
# ament_target_dependencies(rotation_transform_tests
#   ${PROJECT_NAME}
#   gtest_main
# )

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
            include/${PROJECT_NAME}/
        DESTINATION
            include/${PROJECT_NAME}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)


ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ocs2_core ocs2_oc)
ament_package()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_robotic_tools

# ament_add_gtest(rotation_transform_tests
#   test/common/TestRotationTransforms.cpp
#   test/common/TestRotationDerivativesTransforms.cpp
# )
# target_compile_options(rotation_transform_tests PRIVATE ${OCS2_CXX_FLAGS})
