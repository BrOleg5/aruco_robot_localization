add_executable(ArucoLocalizationTest main_test.cpp)

target_link_libraries(ArucoLocalizationTest PUBLIC ArucoLocalizationLib)

# target_include_directories(ArucoLocalizationTest
                           # PUBLIC "${ArucoLocalization_INCLUDE_DIRS}")

if(INSTALL_TESTS)
        install(TARGETS ArucoLocalizationTest
                EXPORT ArucoLocalizationTargets
                RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/ArucoLocalization/tests" COMPONENT bin)
endif()