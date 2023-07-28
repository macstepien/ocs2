cmake_minimum_required(VERSION 3.8)
project (ocs2_frank_wolfe)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(GLPK MODULE REQUIRED)

###########
## Build ##
###########

## Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/FrankWolfeDescentDirection.cpp
  src/GradientDescent.cpp
)
target_link_libraries(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

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
ament_add_gtest(quadratic_test
  test/QuadraticTest.cpp
)
target_link_libraries(quadratic_test
  ${PROJECT_NAME}
  GLPK::GLPK
  gtest_main
)

ament_add_gtest(matyas_test
  test/MatyasTest.cpp
)
target_link_libraries(matyas_test
  ${PROJECT_NAME}
  GLPK::GLPK
  gtest_main
)

ament_add_gtest(glpk_test
  test/testGLPK.cpp
)
target_link_libraries(glpk_test
  ${PROJECT_NAME}
  GLPK::GLPK
  gtest_main
)
endif()

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(ocs2_core
GLPK)

ament_export_include_directories(include
  ${EIGEN3_INCLUDE_DIRS})

ament_export_libraries(${PROJECT_NAME})

ament_package()
