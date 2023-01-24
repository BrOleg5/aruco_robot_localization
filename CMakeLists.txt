cmake_minimum_required(VERSION 3.0)

include("cmake/read_package_version.cmake")

read_package_version_from_xml() # macro from cmake/macros.cmake
project(aruco_robot_localization VERSION ${READ_PROJECT_VERSION} DESCRIPTION 
  "Library to localize mobile robot with atuco marker by static webcamera.")

include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS_RELEASE "/O2")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/O2")
endif()

include(GNUInstallDirs)

option(BUILD_APP "Build application to localize Aruco marker." OFF)
option(BUILD_TEST "Build test programs." OFF)

add_subdirectory(src)

if(BUILD_APP)
  add_subdirectory(app)        
endif()

if(BUILD_TEST)
  include(CTest)
  enable_testing()
  add_subdirectory(test)
endif()

include(CMakePackageConfigHelpers)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME} CACHE PATH "Install path of include directories.")
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake
  PATH_VARS INCLUDE_INSTALL_DIR
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file("${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake COMPONENT Development
)

export(EXPORT ${PROJECT_NAME}Targets
  FILE "${PROJECT_BINARY_DIR}/cmake/${PROJECT_NAME}Targets.cmake"
)

# Install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake COMPONENT Development)