cmake_minimum_required(VERSION 3.8)
project(ocs2_ballbot_mpcnet)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_ballbot
  ocs2_ballbot_ros
  ocs2_mpcnet_core
)

find_package(ament_cmake REQUIRED COMPONENTS
  ${AMENT_PACKAGE_DEPENDENCIES}
)

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
  src/BallbotMpcnetDefinition.cpp
  src/BallbotMpcnetInterface.cpp
)
add_dependencies(${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME})

# python bindings
# pybind11_add_module(BallbotMpcnetPybindings SHARED
#   src/BallbotMpcnetPybindings.cpp
# )
# add_dependencies(BallbotMpcnetPybindings
#   ${PROJECT_NAME}
# )
# target_link_libraries(BallbotMpcnetPybindings PRIVATE
#   ${PROJECT_NAME})
# set_target_properties(BallbotMpcnetPybindings
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

# MPC-Net dummy node
add_executable(ballbot_mpcnet_dummy
  src/BallbotMpcnetDummyNode.cpp
)
add_dependencies(ballbot_mpcnet_dummy
  ${PROJECT_NAME}
)
target_link_libraries(ballbot_mpcnet_dummy
  ${PROJECT_NAME})
target_compile_options(ballbot_mpcnet_dummy PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_ballbot_mpcnet")
   add_clang_tooling(
     TARGETS ${PROJECT_NAME} ballbot_mpcnet_dummy
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

# install(TARGETS BallbotMpcnetPybindings
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

install(TARGETS ballbot_mpcnet_dummy
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch policy
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# TODO(areske)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies()

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_package()