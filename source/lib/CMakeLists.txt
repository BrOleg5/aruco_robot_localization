add_library(aruco_robotino_localization_lib arucolocalization.cpp)

find_package( OpenCV REQUIRED )

target_link_libraries(aruco_robotino_localization_lib ${OpenCV_LIBS} )

target_include_directories(aruco_robotino_localization_lib
          INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
          PUBLIC ${OpenCV_INCLUDE_DIRS}
          "${CMAKE_CURRENT_SOURCE_DIR}/../include"
          )