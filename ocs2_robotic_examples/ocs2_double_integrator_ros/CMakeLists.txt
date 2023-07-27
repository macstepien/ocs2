cmake_minimum_required(VERSION 3.8)
project(ocs2_double_integrator_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_double_integrator
  ocs2_robotic_tools
  ocs2_robotic_assets
)

find_package(ament_cmake REQUIRED COMPONENTS
  roslib
  ${AMENT_PACKAGE_DEPENDENCIES}
)

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

# Mpc node
add_executable(double_integrator_mpc
  src/DoubleIntegratorMpcNode.cpp
)
add_dependencies(double_integrator_mpc
)
target_link_libraries(double_integrator_mpc
)

# Dummy node
add_executable(double_integrator_dummy_test
  src/DummyDoubleIntegratorNode.cpp
  src/DoubleIntegratorDummyVisualization.cpp
)
add_dependencies(double_integrator_dummy_test
)
target_link_libraries(double_integrator_dummy_test
)

# Target node
add_executable(double_integrator_target
  src/DoubleIntegratorTargetPoseCommand.cpp
)
add_dependencies(double_integrator_target
)
target_link_libraries(double_integrator_target
  ${Boost_LIBRARIES}
)
target_compile_options(double_integrator_target PRIVATE ${OCS2_CXX_FLAGS})


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling.")
  add_clang_tooling(
    TARGETS
      double_integrator_mpc
      double_integrator_dummy_test
      double_integrator_target
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
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
    double_integrator_mpc
    double_integrator_dummy_test
    double_integrator_target
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()