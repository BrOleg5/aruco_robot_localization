add_executable(ArucoLocalizationTest main_test.cpp)

# set test dir
if(WIN32 AND NOT CYGWIN)
  set(DEF_TEST_DIR ${CMAKE_INSTALL_BINDIR}/test)
else()
  set(DEF_TEST_DIR ${CMAKE_INSTALL_BINDIR}/ArucoLocalization/test)
endif()
set(TEST_DIR ${DEF_TEST_DIR} CACHE PATH
  "Installation directory for executable files")

target_link_libraries(ArucoLocalizationTest PUBLIC ArucoLocalizationLib)

# target_include_directories(ArucoLocalizationTest
                           # PUBLIC "${ArucoLocalization_INCLUDE_DIRS}")

if(INSTALL_TESTS)
        install(TARGETS ArucoLocalizationTest
                EXPORT ArucoLocalizationTargets
                RUNTIME DESTINATION "${TEST_DIR}" COMPONENT bin)
endif()