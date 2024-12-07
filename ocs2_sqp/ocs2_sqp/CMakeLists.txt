cmake_minimum_required(VERSION 3.5)
project(ocs2_sqp)

find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)

set(AMENT_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
  blasfeo_catkin
  hpipm_catkin
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

###########
## Build ##
###########

# Multiple shooting solver library
add_library(${PROJECT_NAME}
  src/SqpSettings.cpp
  src/SqpSolver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# ament_target_dependencies(test_${PROJECT_NAME}
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
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Testing ##
#############

# ament_add_gtest(test_${PROJECT_NAME}
#   test/testCircularKinematics.cpp
#   test/testSwitchedProblem.cpp
#   test/testUnconstrained.cpp
#   test/testValuefunction.cpp
# )

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${AMENT_DEPENDENCIES})
ament_package()