add_library(arucolocalization arucolocalization.cpp)

find_package(OpenCV 4.2.0 REQUIRED core aruco highgui imgproc videoio)

set_target_properties(arucolocalization PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER "arucolocalization.hpp;${PROJECT_SOURCE_DIR}/include/read_save_camera_parameters.hpp"
                      DEBUG_POSTFIX d)

target_link_libraries(arucolocalization ${OpenCV_LIBS})

target_include_directories(arucolocalization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS arucolocalization
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT Development
  LIBRARY DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}" COMPONENT Development
)