cmake_minimum_required(VERSION 3.8)
project(ocs2_ballbot)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  # pybind11_catkin
  ocs2_core
  ocs2_mpc
  ocs2_ddp
  ocs2_slp
  ocs2_sqp
  ocs2_robotic_tools
  ocs2_python_interface
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

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Ballbot interface library
add_library(${PROJECT_NAME}
  src/BallbotInterface.cpp
  src/dynamics/BallbotSystemDynamics.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# python bindings
# pybind11_add_module(BallbotPyBindings SHARED
#   src/pyBindModule.cpp
# )
# add_dependencies(BallbotPyBindings
#   ${PROJECT_NAME}
# )
# target_link_libraries(BallbotPyBindings PRIVATE
#   ${PROJECT_NAME}
# set_target_properties(BallbotPyBindings
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ballbot")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
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

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# install(TARGETS BallbotPyBindings
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

#############
## Testing ##
#############

# if(BUILD_TESTING)
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(test_BallbotPyBindings
#   test/testBallbotPyBindings.cpp
# )
# target_include_directories(test_BallbotPyBindings PRIVATE
#   ${PROJECT_BINARY_DIR}/include
# )
# target_link_libraries(test_BallbotPyBindings
#   ${PROJECT_NAME}
#   ${Boost_LIBRARIES}
# )
# # python tests
# ament_add_nosetests(test)
# endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(Boost ${AMENT_PACKAGE_DEPENDENCIES})

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()