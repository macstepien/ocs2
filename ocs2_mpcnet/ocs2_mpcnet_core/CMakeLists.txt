cmake_minimum_required(VERSION 3.8)
project(ocs2_mpcnet_core)

set(AMENT_PACKAGE_DEPENDENCIES
  # pybind11_catkin
  ament_cmake
  ocs2_mpc
  ocs2_python_interface
  ocs2_ros_interfaces
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(onnxruntime REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

include_directories(
  include
)

# main library
add_library(${PROJECT_NAME}
  src/control/MpcnetBehavioralController.cpp
  src/control/MpcnetOnnxController.cpp
  src/dummy/MpcnetDummyLoopRos.cpp
  src/dummy/MpcnetDummyObserverRos.cpp
  src/rollout/MpcnetDataGeneration.cpp
  src/rollout/MpcnetPolicyEvaluation.cpp
  src/rollout/MpcnetRolloutBase.cpp
  src/rollout/MpcnetRolloutManager.cpp
  src/MpcnetInterfaceBase.cpp
)
add_dependencies(${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}
  onnxruntime
)

# python bindings
# pybind11_add_module(MpcnetPybindings SHARED
#   src/MpcnetPybindings.cpp
# )
# add_dependencies(MpcnetPybindings
#   ${PROJECT_NAME}
# )
# target_link_libraries(MpcnetPybindings PRIVATE
#   ${PROJECT_NAME}
# set_target_properties(MpcnetPybindings
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_mpcnet_core")
   add_clang_tooling(
     TARGETS ${PROJECT_NAME}
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
        DESTINATION include/${PROJECT_NAME})

# install(TARGETS MpcnetPybindings
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

#############
## Testing ##
#############

# TODO(areske)


###################################
## ament specific configuration ##
###################################

ament_export_dependencies(onnxruntime ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package()