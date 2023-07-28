cmake_minimum_required(VERSION 3.8)
project(ocs2_raisim_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  roscpp
  ocs2_raisim_core
  grid_map_msgs
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########
include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/RaisimHeightmapRosConverter.cpp
)
target_link_libraries(${PROJECT_NAME}
)

#######################
##   CLANG TOOLING   ##
#######################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for project " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
    CT_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/testRosConversion.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    gtest_main
  )
endif()
###################################
## ament specific configuration ##
###################################

ament_export_dependencies( ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package()