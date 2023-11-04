cmake_minimum_required(VERSION 3.8)
project(ocs2_mobile_manipulator_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  rclcpp
  # tf
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  geometry_msgs
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  # ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_self_collision_visualization
  ocs2_mobile_manipulator
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# MPC node
# add_executable(mobile_manipulator_mpc_node
#   src/MobileManipulatorMpcNode.cpp
# )
# ament_target_dependencies(mobile_manipulator_mpc_node
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(mobile_manipulator_mpc_node
#   ${Boost_LIBRARIES}
# )
# target_compile_options(mobile_manipulator_mpc_node PUBLIC ${FLAGS})

# DistanceVisualization node
# add_executable(mobile_manipulator_distance_visualization
#   src/MobileManipulatorDistanceVisualization.cpp
# )
# ament_target_dependencies(mobile_manipulator_distance_visualization
#   ${AMENT_PACKAGE_DEPENDENCIES}
# )
# target_link_libraries(mobile_manipulator_distance_visualization
# )
# target_compile_options(mobile_manipulator_distance_visualization PUBLIC ${FLAGS})

# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
  src/MobileManipulatorDummyMRT.cpp
  src/MobileManipulatorDummyVisualization.cpp
)
ament_target_dependencies(mobile_manipulator_dummy_mrt_node
  rclcpp ocs2_mobile_manipulator ocs2_ros_interfaces ocs2_mpc ocs2_ddp urdf
  robot_state_publisher ocs2_self_collision_visualization
)
target_link_libraries(mobile_manipulator_dummy_mrt_node
)
target_compile_options(mobile_manipulator_dummy_mrt_node PUBLIC ${FLAGS})

# Target node
add_executable(mobile_manipulator_target
  src/MobileManipulatorTarget.cpp
)
ament_target_dependencies(mobile_manipulator_target
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(mobile_manipulator_target
)
target_compile_options(mobile_manipulator_target PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS mobile_manipulator_mpc_node mobile_manipulator_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS 
    # mobile_manipulator_mpc_node 
    # mobile_manipulator_distance_visualization
    mobile_manipulator_dummy_mrt_node 
    mobile_manipulator_target
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost ${AMENT_PACKAGE_DEPENDENCIES}
pinocchio)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()