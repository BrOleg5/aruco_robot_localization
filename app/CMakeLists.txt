add_executable(localization localization.cpp)

find_package(SharedMemory 1.2 REQUIRED)

target_link_libraries(localization PUBLIC
                      arucolocalization
                      SharedMemoryLib)

target_include_directories(localization PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS localization
  EXPORT ArucoLocalizationTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
)

# # set header
# set(HEADER ${PROJECT_SOURCE_DIR}/include/cmdoptionparser.hpp)
# install(FILES ${HEADER} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)

add_executable(generate_marker generate_marker.cpp)

find_package(OpenCV 4.2.0 REQUIRED)

target_link_libraries(generate_marker ${OpenCV_LIBS})

target_include_directories(generate_marker PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS generate_marker
        EXPORT ArucoLocalizationTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime)



add_executable(detect_aruco_marker detect_aruco_marker.cpp)

target_link_libraries(detect_aruco_marker PUBLIC arucolocalization)

target_include_directories(detect_aruco_marker PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS detect_aruco_marker
  EXPORT ArucoLocalizationTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT Runtime
)

add_subdirectory("camera calibration")