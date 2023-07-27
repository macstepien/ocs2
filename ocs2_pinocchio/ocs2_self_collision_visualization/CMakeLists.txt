cmake_minimum_required(VERSION 3.8)
project(ocs2_self_collision_visualization)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_ros_interfaces

  # visualization deps
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

find_package(ament_cmake REQUIRED COMPONENTS
  ${AMENT_PACKAGE_DEPENDENCIES}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)
# requires liboctomap-dev and libassimp-dev

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
  ${pinocchio_INCLUDE_DIRS}
  ${hpp-fcl_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
  src/GeometryInterfaceVisualization.cpp
)
add_dependencies(${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
      TARGETS ${PROJECT_NAME}
      SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
      CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
      CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(pinocchio)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package()