cmake_minimum_required(VERSION 3.8)
project(ocs2_robotic_tools)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_core REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${ocs2_core_INCLUDE_DIRS}
  ${ocs2_oc_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/common/RotationTransforms.cpp
  src/common/LoopshapingRobotInterface.cpp
)
target_link_libraries(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
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

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
            include/${PROJECT_NAME}/
        DESTINATION
            include/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(rotation_transform_tests
  test/common/TestRotationTransforms.cpp
  test/common/TestRotationDerivativesTransforms.cpp
)
target_link_libraries(rotation_transform_tests
  ${PROJECT_NAME}
  gtest_main
)
target_compile_options(rotation_transform_tests PRIVATE ${OCS2_CXX_FLAGS})
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(ocs2_core
ocs2_oc)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()