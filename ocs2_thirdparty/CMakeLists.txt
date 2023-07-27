cmake_minimum_required(VERSION 3.8)
project(ocs2_thirdparty)

find_package(ament_cmake)

install(DIRECTORY include/
        DESTINATION "include/${PROJECT_NAME}/../")

###################################
## ament specific configuration ##
###################################

ament_export_include_directories(include)

ament_package()