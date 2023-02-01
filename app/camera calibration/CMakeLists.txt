set(INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/camera calibration")

find_package(OpenCV 4.2.0 REQUIRED)

add_executable(calibrate_camera calibrate_camera.cpp)

target_link_libraries(calibrate_camera PUBLIC
  arucolocalization
  ${OpenCV_LIBS}
)

target_include_directories(calibrate_camera PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS calibrate_camera
  EXPORT ArucoLocalizationTargets
  RUNTIME DESTINATION ${INSTALL_DESTINATION} COMPONENT bin
)