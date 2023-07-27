cmake_minimum_required(VERSION 3.8)
project(ocs2_msgs)

find_package(ament_cmake REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  FILES
    mpc_state.msg
    mpc_input.msg
    mode_schedule.msg
    mpc_observation.msg
    mpc_performance_indices.msg
    mpc_target_trajectories.msg
    controller_data.msg
    mpc_flattened_controller.msg
    lagrangian_metrics.msg
    multiplier.msg
    constraint.msg
)

add_service_files(
  FILES
    reset.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(std_msgs
message_runtime)

ament_package()