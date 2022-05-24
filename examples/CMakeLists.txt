add_executable(get_position get_position.cpp)

target_link_libraries(get_position PUBLIC arucolocalization)

install(TARGETS get_position
        EXPORT ArucoLocalizationTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/examples" COMPONENT bin)