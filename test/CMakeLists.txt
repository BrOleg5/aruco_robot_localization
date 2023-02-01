add_executable(global_coordinates_test global_coordinates_test.cpp)

find_package(OpenCV 4.2.0 REQUIRED)

target_link_libraries(global_coordinates_test PUBLIC
  arucolocalization
  ${OpenCV_LIBS}
)

target_include_directories(global_coordinates_test PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${OpenCV_INCLUDE_DIRS}
)

set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME GlobalCoordinateTest
  COMMAND global_coordinates_test -d=0 --id=0 --img=${TEST_DIR}/test_frames --cp=${TEST_DIR}/camera_params.json)
set_tests_properties(GlobalCoordinateTest PROPERTIES
  PASS_REGULAR_EXPRESSION "Test success."
  FAIL_REGULAR_EXPRESSION "Test failed."
)

install(TARGETS global_coordinates_test
  EXPORT ArucoLocalizationTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/test" COMPONENT bin
)