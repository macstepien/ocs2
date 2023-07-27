cmake_minimum_required(VERSION 3.8)
project(ocs2_sqp)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  ocs2_core
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
  blasfeo_catkin
  hpipm_catkin
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

# Multiple shooting solver library
add_library(${PROJECT_NAME}
  src/SqpSettings.cpp
  src/SqpSolver.cpp
)
# add_dependencies(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
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

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_${PROJECT_NAME}
  test/testCircularKinematics.cpp
  test/testSwitchedProblem.cpp
  test/testUnconstrained.cpp
  test/testValuefunction.cpp
)
add_dependencies(test_${PROJECT_NAME}
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  gtest_main
)
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(
Boost)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()