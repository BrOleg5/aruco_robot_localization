# source: https://github.com/MarcTestier/ros2_cmake_test/blob/master/foo/CMakeLists.txt
# Read version from the package.xml file.
macro(read_package_version_from_xml)
  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/package.xml package_xml_str)
  if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
    message(FATAL_ERROR "Could not parse project version from package.xml.")
  endif()
  set(READ_PROJECT_VERSION ${CMAKE_MATCH_1})
endmacro()