cmake_minimum_required(VERSION 3.5)
project(ocs2_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 1 4)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/MpcPerformanceIndices.msg
        msg/MpcObservation.msg
        msg/LagrangianMetrics.msg
        msg/Multiplier.msg
        msg/ModeSchedule.msg
        msg/MpcInput.msg
        msg/MpcState.msg
        msg/MpcTargetTrajectories.msg
        msg/Constraint.msg
        msg/MpcFlattenedController.msg
        msg/ControllerData.msg
        srv/Reset.srv
    DEPENDENCIES
        
)

ament_package()
