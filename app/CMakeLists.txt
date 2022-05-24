add_executable(localization localization.cpp)

find_package(SharedMemory 1.2 REQUIRED)

target_link_libraries(localization PUBLIC
                      ArucoLocalizationLib
                      SharedMemoryLib)

install(TARGETS localization
        EXPORT ArucoLocalizationTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)