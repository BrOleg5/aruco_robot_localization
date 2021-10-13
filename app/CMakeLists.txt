add_executable(localization localization.cpp)

find_package(SharedMemory 1.1 REQUIRED)

target_link_libraries(localization PUBLIC
                      ArucoLocalizationLib
                      SharedMemoryLib)

# target_include_directories(localization
                         #   PUBLIC ${ArucoLocalization_INCLUDE_DIRS}
                         #   ${SharedMemory_INCLUDE_DIRS})

if(INSTALL_APP)
    install(TARGETS localization
            EXPORT ArucoLocalizationTargets
            RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/ArucoLocalization" COMPONENT bin)
endif()