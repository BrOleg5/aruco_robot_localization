cmake_minimum_required(VERSION 3.16)

# set the project name and version
project(aruco_robotino_localization_lib VERSION 1.2 DESCRIPTION "Library to localize mobile robot with atuco marker by static webcamera.")

add_subdirectory(app)
add_subdirectory(tests)

include(GNUInstallDirs)

add_library(aruco_robotino_localization_lib src/arucolocalization.cpp)

find_package( OpenCV REQUIRED )

set_target_properties(aruco_robotino_localization_lib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER include/arucolocalization.hpp)

target_include_directories(aruco_robotino_localization_lib 
                           PRIVATE include
                           PRIVATE src
                           PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(aruco_robotino_localization_lib ${OpenCV_LIBS} )

install(TARGETS aruco_robotino_localization_lib
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)