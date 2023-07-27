cmake_minimum_required(VERSION 3.8)
project(ocs2_python_interface)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  ocs2_mpc
  ocs2_ddp
  ocs2_robotic_tools
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/PythonInterface.cpp
)

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

#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(testDummyPyBindings
  test/testDummyPyBindings.cpp
)
target_link_libraries(testDummyPyBindings
  ${PROJECT_NAME}
  gtest_main
)
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies()

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()