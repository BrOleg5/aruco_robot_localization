add_executable(main_test main_test.cpp)

target_link_libraries(main_test PUBLIC aruco_robotino_localization_lib)

target_include_directories(main_test
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")