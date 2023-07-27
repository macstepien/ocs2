cmake_minimum_required(VERSION 3.8)
project(ocs2_ros_interfaces)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  roscpp
  ocs2_msgs
  ocs2_core
  ocs2_mpc
  std_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS filesystem)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/command/TargetTrajectoriesRosPublisher.cpp
  src/command/TargetTrajectoriesInteractiveMarker.cpp
  src/command/TargetTrajectoriesKeyboardPublisher.cpp
  src/common/RosMsgConversions.cpp
  src/common/RosMsgHelpers.cpp
  src/mpc/MPC_ROS_Interface.cpp
  src/mrt/LoopshapingDummyObserver.cpp
  src/mrt/MRT_ROS_Dummy_Loop.cpp
  src/mrt/MRT_ROS_Interface.cpp
  src/synchronized_module/RosReferenceManager.cpp
  src/synchronized_module/SolverObserverRosCallbacks.cpp
  src/visualization/VisualizationHelpers.cpp
  src/visualization/VisualizationColors.cpp
)
target_link_libraries(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(test_custom_callback_queue
  test/test_custom_callback_queue.cpp
)
add_dependencies(test_custom_callback_queue
)
target_link_libraries(test_custom_callback_queue
)
target_compile_options(test_custom_callback_queue PRIVATE ${OCS2_CXX_FLAGS})

# multiplot remap node
add_executable(multiplot_remap
  src/multiplot/MultiplotRemap.cpp
)
add_dependencies(multiplot_remap
)
target_link_libraries(multiplot_remap
  ${PROJECT_NAME}
target_compile_options(multiplot_remap PRIVATE ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
add_dependencies(${PROJECT_NAME}_lintTarget
)
target_link_libraries(${PROJECT_NAME}_lintTarget
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
  TARGETS ${PROJECT_NAME}_lintTarget
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
install(TARGETS multiplot_remap
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch multiplot
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()