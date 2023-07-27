cmake_minimum_required(VERSION 3.8)
project(ocs2_legged_robot_raisim)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_legged_robot
  ocs2_legged_robot_ros
  ocs2_raisim_core
  ocs2_raisim_ros
)

find_package(ament_cmake REQUIRED COMPONENTS
  ${AMENT_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

# Add directories for all targets
include_directories(
  include
)

# main library
add_library(${PROJECT_NAME}
  src/LeggedRobotRaisimConversions.cpp
  src/LeggedRobotRaisimVisualizer.cpp
)
add_dependencies(${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# RaiSim dummy
add_executable(legged_robot_raisim_dummy
  src/LeggedRobotRaisimDummyNode.cpp
)
add_dependencies(legged_robot_raisim_dummy
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_raisim_dummy
  ${PROJECT_NAME}
target_compile_options(legged_robot_raisim_dummy PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ballbot")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME} legged_robot_raisim_dummy
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
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

install(TARGETS legged_robot_raisim_dummy
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_test
  test/testLeggedRobotRaisimConversions.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME})
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies()

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package()