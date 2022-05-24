add_executable(localization localization.cpp)

find_package(SharedMemory 1.2 REQUIRED)

target_link_libraries(localization PUBLIC
                      arucolocalization
                      SharedMemoryLib)

target_include_directories(localization PUBLIC 
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(TARGETS localization
        EXPORT ArucoLocalizationTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

# set header
set(HEADER ${PROJECT_SOURCE_DIR}/include/cmdoptionparser.hpp)
install(FILES ${HEADER} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT dev)