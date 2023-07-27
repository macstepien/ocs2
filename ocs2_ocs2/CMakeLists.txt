cmake_minimum_required(VERSION 3.8)
project(ocs2_ocs2)

find_package(ament_cmake REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  ocs2_ddp
  ocs2_frank_wolfe
)
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
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_link_libraries(${PROJECT_NAME}_lintTarget
  ${Boost_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ocs2")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#############
## Testing ##
#############

#ament_add_gtest(exp0_gddp_test
#  test/exp0_gddp_test.cpp
#)
#target_link_libraries(exp0_gddp_test
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp0_gddp_test PRIVATE ${OCS2_CXX_FLAGS})

#ament_add_gtest(exp1_gddp_test
#  test/exp1_gddp_test.cpp
#)
#target_link_libraries(exp1_gddp_test
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp1_gddp_test PRIVATE ${OCS2_CXX_FLAGS})

#ament_add_gtest(exp0_ocs2_test
#  test/exp0_ocs2_test.cpp
#)
#target_link_libraries(exp0_ocs2_test
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp0_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})

#ament_add_gtest(exp1_ocs2_test
#  test/exp1_ocs2_test.cpp
#)
#target_link_libraries(exp1_ocs2_test
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp1_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost
ocs2_core
ocs2_ddp
ocs2_frank_wolfe)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()