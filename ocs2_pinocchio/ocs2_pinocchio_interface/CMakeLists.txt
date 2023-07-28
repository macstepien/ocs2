cmake_minimum_required(VERSION 3.8)
project(ocs2_pinocchio_interface)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  ocs2_core
  ocs2_robotic_tools
)



foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add pinocchio configurations
include(cmake/pinocchio_config.cmake)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
  src/PinocchioInterface.cpp
  src/PinocchioInterfaceCppAd.cpp
  src/PinocchioEndEffectorKinematics.cpp
  src/PinocchioEndEffectorKinematicsCppAd.cpp
  src/urdf.cpp
)
add_dependencies(${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

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

############
# Testing ##
############
if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(testPinocchioInterface
  test/testPinocchioInterface.cpp
  test/testPinocchioEndEffectorKinematics.cpp
)
target_link_libraries(testPinocchioInterface
  gtest_main
  ${PROJECT_NAME}
endif()
###################################
## ament specific configuration ##
###################################

ament_export_dependencies(pinocchio ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include
  ${pinocchio_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS
    cmake/pinocchio_config.cmake
)