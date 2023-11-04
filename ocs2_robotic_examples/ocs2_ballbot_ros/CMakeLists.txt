cmake_minimum_required(VERSION 3.8)
project(ocs2_ballbot_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  tf
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_slp
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_ballbot
  ocs2_robotic_assets
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

# main library
add_library(${PROJECT_NAME}
  src/BallbotDummyVisualization.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Mpc node
add_executable(ballbot_ddp
  src/BallbotDdpMpcNode.cpp
)
target_link_libraries(ballbot_ddp
)
target_compile_options(ballbot_ddp PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(ballbot_dummy_test
  src/DummyBallbotNode.cpp
)
add_dependencies(ballbot_dummy_test
  ${PROJECT_NAME}
)
target_link_libraries(ballbot_dummy_test
  ${PROJECT_NAME}
)
target_compile_options(ballbot_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

# Target node
add_executable(ballbot_target
  src/BallbotTargetPoseCommand.cpp
)
target_link_libraries(ballbot_target
  ${Boost_LIBRARIES}
)
target_compile_options(ballbot_target PRIVATE ${OCS2_CXX_FLAGS})

## Combined MPC MRT example
add_executable(ballbot_mpc_mrt
  src/BallbotMpcMrtNode.cpp
  src/BallbotDummyVisualization.cpp
  )
target_link_libraries(ballbot_mpc_mrt
  )
target_compile_options(ballbot_mpc_mrt  PRIVATE ${OCS2_CXX_FLAGS})

## SQP node for ballbot
add_executable(ballbot_sqp
  src/BallbotSqpMpcNode.cpp
)
target_link_libraries(ballbot_sqp
)
target_compile_options(ballbot_sqp  PRIVATE ${OCS2_CXX_FLAGS})

## SLP node for ballbot
add_executable(ballbot_slp
  src/BallbotSlpMpcNode.cpp
)
target_link_libraries(ballbot_slp
)
target_compile_options(ballbot_slp  PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ballbot")
  add_clang_tooling(
    TARGETS
        ${PROJECT_NAME}
        ballbot_ddp
        ballbot_sqp
        ballbot_dummy_test
        ballbot_target
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
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
install(TARGETS ballbot_ddp ballbot_dummy_test ballbot_target ballbot_sqp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)


#############
## Testing ##
#############

# if(BUILD_TESTING)
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(ballbot_integration_test
#   test/ballbotIntegrationTest.cpp
# )
# target_link_libraries(ballbot_integration_test
#   ${Boost_LIBRARIES}
# )
# endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()