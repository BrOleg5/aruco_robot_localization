cmake_minimum_required(VERSION 3.0)

project(ArucoLocalization VERSION 1.3 DESCRIPTION "Library to localize mobile robot with atuco marker by static webcamera.")

# set cmake config install dirs
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/ArucoLocalization)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

option(BUILD_APP "Build application to localize Aruco marker." OFF)
option(INSTALL_APP "Install application to localize Aruco marker" OFF)
option(BUILD_TESTS "Build tests programs." OFF)
option(INSTALL_TESTS "Install tests programs." OFF)

add_subdirectory(src)

if(BUILD_APP OR INSTALL_APP)
        add_subdirectory(app)        
endif()

if(BUILD_TESTS OR INSTALL_TESTS)
        add_subdirectory(tests) 
endif()

# Add all targets to the build-tree export set
if(BUILD_TESTS OR INSTALL_TESTS)
  if(BUILD_APP OR INSTALL_APP)
        export(TARGETS ArucoLocalizationLib ArucoLocalizationTest localization
               FILE "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationTargets.cmake")
  else()
        export(TARGETS ArucoLocalizationLib ArucoLocalizationTest
               FILE "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationTargets.cmake")
  endif()
else()
  if(BUILD_APP OR INSTALL_APP)
        export(TARGETS ArucoLocalizationLib localization
                FILE "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationTargets.cmake")
  else()
        export(TARGETS ArucoLocalizationLib
                FILE "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationTargets.cmake")
  endif()
endif()

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ArucoLocalization)

# Create the ArucoLocalizationConfig.cmake and ArucoLocalizationConfigVersion.cmake files
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(cmake/ArucoLocalizationConfig.cmake.in
               "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${ARUCOLOCALIZATION_CMAKE_DIR}/${CMAKE_INSTALL_INCLUDEDIR}")
configure_file(cmake/ArucoLocalizationConfig.cmake.in
               "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/cmake/ArucoLocalizationConfig.cmake" @ONLY)

# ... for both
configure_file(cmake/ArucoLocalizationConfigVersion.cmake.in
               "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationConfigVersion.cmake" @ONLY)

# Install the ArucoLocalizationConfig.cmake and ArucoLocalizationConfigVersion.cmake files
install(FILES
        "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationConfig.cmake"
        "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationConfigVersion.cmake"
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT ArucoLocalizationTargets
        DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)