cmake_minimum_required(VERSION 3.8)
project(ocs2_legged_robot)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
)

find_package(ament_cmake REQUIRED COMPONENTS
  ${AMENT_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

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
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Legged robot interface library
add_library(${PROJECT_NAME}
  src/common/ModelSettings.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/reference_manager/SwitchedModelReferenceManager.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/gait/Gait.cpp
  src/gait/GaitSchedule.cpp
  src/gait/LegLogic.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/LeggedRobotInterface.cpp
  src/LeggedRobotPreComputation.cpp
)
add_dependencies(${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot")
   add_clang_tooling(
     TARGETS ${PROJECT_NAME}
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

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
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_test
  test/AnymalFactoryFunctions.cpp
  test/constraint/testEndEffectorLinearConstraint.cpp
  test/constraint/testFrictionConeConstraint.cpp
  test/constraint/testZeroForceConstraint.cpp
)
target_include_directories(${PROJECT_NAME}_test PRIVATE
  test/include
  ${PROJECT_BINARY_DIR}/include
)
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})
endif()
###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost
pinocchio)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()