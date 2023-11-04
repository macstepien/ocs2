cmake_minimum_required(VERSION 3.8)
project(ocs2_quadrotor_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_quadrotor
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# MPC node
add_executable(quadrotor_mpc
  src/QuadrotorMpcNode.cpp
)
target_link_libraries(quadrotor_mpc
)

# Dummy node
add_executable(quadrotor_dummy_test
  src/DummyQuadrotorNode.cpp
  src/QuadrotorDummyVisualization.cpp
)
target_link_libraries(quadrotor_dummy_test
)

# Target node
add_executable(quadrotor_target
  src/QuadrotorTargetPoseCommand.cpp
)
target_link_libraries(quadrotor_target
  ${Boost_LIBRARIES}
)
target_compile_options(quadrotor_target PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_quadrotor")
  add_clang_tooling(
    TARGETS
      quadrotor_mpc
      quadrotor_dummy_test
      quadrotor_target
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
install(
  TARGETS 
    quadrotor_mpc 
    quadrotor_dummy_test 
    quadrotor_target
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()