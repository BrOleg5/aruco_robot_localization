add_library(ArucoLocalizationLib arucolocalization.cpp)

find_package(OpenCV REQUIRED)

# set header
set(HEADER ${PROJECT_SOURCE_DIR}/include/cmdoptionparser.hpp)

# set include and lib dirs
if(WIN32 AND NOT CYGWIN)
  set(DEF_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})
else()
  set(DEF_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR}/ArucoLocalization)
endif()
set(INCLUDE_DIR ${DEF_INCLUDE_DIR} CACHE PATH
  "Installation directory for include files")

set_target_properties(ArucoLocalizationLib PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      PUBLIC_HEADER arucolocalization.hpp)

target_link_libraries(ArucoLocalizationLib ${OpenCV_LIBS})

target_include_directories(ArucoLocalizationLib PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${INCLUDE_DIR}/ArucoLocalization>
                           "${OpenCV_INCLUDE_DIRS}")

install(TARGETS ArucoLocalizationLib
        EXPORT ArucoLocalizationTargets
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        PUBLIC_HEADER DESTINATION "${INCLUDE_DIR}" COMPONENT dev)
install(FILES ${HEADER} DESTINATION "${INCLUDE_DIR}")