cmake_minimum_required(VERSION 3.0)

set(ARUCOLOCALIZATION_MAJOR_VERSION 1)
set(ARUCOLOCALIZATION_MINOR_VERSION 4)
set(ARUCOLOCALIZATION_PATCH_VERSION 3)
set(ARUCOLOCALIZATION_VERSION 
    ${ARUCOLOCALIZATION_MAJOR_VERSION}.${ARUCOLOCALIZATION_MINOR_VERSION}.${ARUCOLOCALIZATION_PATCH_VERSION})

project(ArucoLocalization VERSION ${ARUCOLOCALIZATION_VERSION} DESCRIPTION 
        "Library to localize mobile robot with atuco marker by static webcamera.")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif()

# set cmake config install dirs
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR cmake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/ArucoLocalization)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

include(GNUInstallDirs)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

option(BUILD_APP "Build application to localize Aruco marker." OFF)
option(BUILD_EXAMPLES "Build and install examples." OFF)

add_subdirectory(src)

if(BUILD_APP)
        add_subdirectory(app)        
endif()

if(BUILD_EXAMPLES)
        add_subdirectory(examples)
endif()

# Add all targets to the build-tree export set
export(TARGETS arucolocalization
       FILE "${PROJECT_BINARY_DIR}/cmake/ArucoLocalizationTargets.cmake")

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