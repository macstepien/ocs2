cmake_minimum_required(VERSION 3.14)
project(hpipm_catkin)

set(AMENT_PACKAGE_DEPENDENCIES
  ament_cmake
  blasfeo_catkin
  ocs2_core
  ocs2_oc
  ocs2_qp_solver
)

foreach(Dependency IN ITEMS ${AMENT_PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(FetchContent)

# Define directories
set(HPIPM_DEVEL_PREFIX share CACHE STRING "HPIPM install path")
set(HPIPM_INCLUDE_DIR ${HPIPM_DEVEL_PREFIX}/include)
set(HPIPM_LIB_DIR ${HPIPM_DEVEL_PREFIX}/lib)
set(HPIPM_DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/download)
set(HPIPM_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build)

# Create directories if they do not exist
file(MAKE_DIRECTORY ${HPIPM_INCLUDE_DIR})
file(MAKE_DIRECTORY ${HPIPM_LIB_DIR})
file(MAKE_DIRECTORY ${HPIPM_DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${HPIPM_BUILD_DIR})

message(WARNING "BLASFEO_PATH: " ${BLASFEO_PATH})

# HPIPM Settings
set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(hpipmDownload
  GIT_REPOSITORY https://github.com/giaf/hpipm
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
  UPDATE_COMMAND ""
  SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
  BINARY_DIR ${HPIPM_BUILD_DIR}
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND "$(MAKE) install"
  )
FetchContent_MakeAvailable(hpipmDownload)

file(GLOB_RECURSE HEADERS "${HPIPM_DOWNLOAD_DIR}/include/*.h")
foreach(HEADER_FILE ${HEADERS})
  message(STATUS "FOUND HEADER: " ${HEADER_FILE})
  file(COPY ${HEADER_FILE} DESTINATION ${HPIPM_INCLUDE_DIR})
endforeach()

set_target_properties(hpipm PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${HPIPM_LIB_DIR})

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Hpipm interface
add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)
# add_dependencies(${PROJECT_NAME}
# )
ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGE_DEPENDENCIES}
)
target_link_libraries(${PROJECT_NAME}
  hpipm
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

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(test_${PROJECT_NAME}
  test/testHpipmInterface.cpp
)
add_dependencies(test_${PROJECT_NAME})
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  hpipm
  gtest_main
)
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(blasfeo_catkin)

ament_export_include_directories(include
  ${HPIPM_INCLUDE_DIR})

ament_export_libraries(${PROJECT_NAME} hpipm)

ament_package()