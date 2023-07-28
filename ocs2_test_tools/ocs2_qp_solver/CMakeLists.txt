cmake_minimum_required(VERSION 3.8)
project(ocs2_qp_solver)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  ocs2_core
  ocs2_oc
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
  
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
  include
  test/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/Ocs2QpSolver.cpp
  src/QpDiscreteTranscription.cpp
  src/QpSolver.cpp
  src/QpTrajectories.cpp
)
target_link_libraries(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
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

install(DIRECTORY test/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_${PROJECT_NAME}
  test/testDiscreteTranscription.cpp
  test/testQpSolver.cpp
  test/testOcs2QpSolver.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  gtest_main
)
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(ocs2_core
ocs2_oc)

ament_export_include_directories(include
  # test/include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()