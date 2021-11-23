add_executable(localization localization.cpp)

find_package(SharedMemory 1.2 REQUIRED)

# set test dir
if(WIN32 AND NOT CYGWIN)
  set(DEF_BIN_DIR ${CMAKE_INSTALL_BINDIR})
else()
  set(DEF_BIN_DIR ${CMAKE_INSTALL_BINDIR}/ArucoLocalization)
endif()
set(BIN_DIR ${DEF_BIN_DIR} CACHE PATH
  "Installation directory for executable files")

target_link_libraries(localization PUBLIC
                      ArucoLocalizationLib
                      SharedMemoryLib)

# target_include_directories(localization
                         #   PUBLIC ${ArucoLocalization_INCLUDE_DIRS}
                         #   ${SharedMemory_INCLUDE_DIRS})

if(INSTALL_APP)
    install(TARGETS localization
            EXPORT ArucoLocalizationTargets
            RUNTIME DESTINATION "${BIN_DIR}" COMPONENT bin)
endif()