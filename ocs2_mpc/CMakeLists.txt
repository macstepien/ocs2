cmake_minimum_required(VERSION 3.8)
project(ocs2_mpc)

set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ocs2_core
  ocs2_oc
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${ocs2_core_INCLUDE_DIRS}
  ${ocs2_oc_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/LoopshapingSystemObservation.cpp
  src/MPC_BASE.cpp
  src/MPC_Settings.cpp
  src/SystemObservation.cpp
  src/MRT_BASE.cpp
  src/MPC_MRT_Interface.cpp
  # src/MPC_OCS2.cpp
)
target_link_libraries(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
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

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

#############
## Testing ##
#############

#ament_add_gtest(testMPC_OCS2
#  test/testMPC_OCS2.cpp
#)
#target_link_libraries(testMPC_OCS2
#  ${Boost_LIBRARIES}
#)
#target_compile_options(testMPC_OCS2 PRIVATE ${OCS2_CXX_FLAGS})

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(ocs2_core
ocs2_oc
Boost)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()