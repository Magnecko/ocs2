cmake_minimum_required(VERSION 3.5)
project(ocs2_oc)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/approximate_model/ChangeOfInputVariables.cpp
  src/approximate_model/LinearQuadraticApproximator.cpp
  src/multiple_shooting/Helpers.cpp
  src/multiple_shooting/Initialization.cpp
  src/multiple_shooting/LagrangianEvaluation.cpp
  src/multiple_shooting/MetricsComputation.cpp
  src/multiple_shooting/PerformanceIndexComputation.cpp
  src/multiple_shooting/ProjectionMultiplierCoefficients.cpp
  src/multiple_shooting/Transcription.cpp
  src/oc_data/LoopshapingPrimalSolution.cpp
  src/oc_data/PerformanceIndex.cpp
  src/oc_data/TimeDiscretization.cpp
  src/oc_problem/OptimalControlProblem.cpp
  src/oc_problem/LoopshapingOptimalControlProblem.cpp
  src/oc_problem/OptimalControlProblemHelperFunction.cpp
  src/oc_problem/OcpSize.cpp
  src/oc_problem/OcpToKkt.cpp
  src/oc_solver/SolverBase.cpp
  src/precondition/Ruzi.cpp
  src/rollout/PerformanceIndicesRollout.cpp
  src/rollout/RolloutBase.cpp
  src/rollout/RootFinder.cpp
  src/rollout/InitializerRollout.cpp
  src/rollout/StateTriggeredRollout.cpp
  src/rollout/TimeTriggeredRollout.cpp
  src/rollout/RolloutSettings.cpp
  src/synchronized_module/ReferenceManager.cpp
  src/synchronized_module/LoopshapingReferenceManager.cpp
  src/synchronized_module/LoopshapingSynchronizedModule.cpp
  src/synchronized_module/SolverObserver.cpp
  src/search_strategy/FilterLinesearch.cpp
  src/trajectory_adjustment/TrajectorySpreading.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ocs2_core
)

target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament_target_dependencies(test_${PROJECT_NAME}_multiple_shooting
#   ${PROJECT_NAME}
#   gtest_main
# )

# ament_target_dependencies(test_${PROJECT_NAME}_data
#   ${PROJECT_NAME}
#   gtest_main
# )

# ament_target_dependencies(test_${PROJECT_NAME}_rollout
#   ${PROJECT_NAME}
#   gtest_main
# )

# ament_target_dependencies(test_change_of_variables
#   ${PROJECT_NAME}
#   gtest_main
# )

# ament_target_dependencies(test_trajectory_spreading
#   ${PROJECT_NAME}
#   gtest_main
# )
# ament_target_dependencies(test_ocp_to_kkt
#   ${PROJECT_NAME}
#   gtest_main
# )
# ament_target_dependencies(test_precondition
#   ${PROJECT_NAME}
#   gtest_main
# )


#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
# install(DIRECTORY test/include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(ocs2_core)
ament_package()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_oc

# ament_add_gtest(test_${PROJECT_NAME}_multiple_shooting
#   test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
#   test/multiple_shooting/testTranscriptionMetrics.cpp
#   test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
# )

# ament_add_gtest(test_${PROJECT_NAME}_data
#   test/oc_data/testTimeDiscretization.cpp
# )

# ament_add_gtest(test_${PROJECT_NAME}_rollout
#    test/rollout/testTimeTriggeredRollout.cpp
#    test/rollout/testStateTriggeredRollout.cpp
# )

# ament_add_gtest(test_change_of_variables
#   test/testChangeOfInputVariables.cpp
# )

# ament_add_gtest(test_trajectory_spreading
#   test/trajectory_adjustment/TrajectorySpreadingTest.cpp
# )

# ament_add_gtest(test_ocp_to_kkt
#   test/oc_problem/testOcpToKkt.cpp
# )

# ament_add_gtest(test_precondition
#   test/precondition/testPrecondition.cpp
# )

