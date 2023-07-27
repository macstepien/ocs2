cmake_minimum_required(VERSION 3.8)
project(ocs2_cartpole_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_cartpole
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
add_executable(cartpole_mpc
  src/CartpoleMpcNode.cpp
)
add_dependencies(cartpole_mpc
)
target_link_libraries(cartpole_mpc
)

# Dummy node
add_executable(cartpole_dummy_test
  src/DummyCartpoleNode.cpp
  src/CartpoleDummyVisualization.cpp
)
add_dependencies(cartpole_dummy_test
)
target_link_libraries(cartpole_dummy_test
)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Running clang tooling.")
  add_clang_tooling(
    TARGETS
        cartpole_mpc
        cartpole_dummy_test
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS cartpole_mpc cartpole_dummy_test
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