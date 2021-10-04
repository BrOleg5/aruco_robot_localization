add_executable(localization localization.cpp)

target_link_libraries(localization 
                      PUBLIC aruco_robotino_localization_lib
                      ${Boost_LIBRARIES} rt
                      ${Boost_LIBRARIES} pthread)

target_include_directories(localization
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include"
        "/usr/local/boost_1_77_0")