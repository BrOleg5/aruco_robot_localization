# Aruco mobile robot localization

Application and library was created to localize mobile robot by Aruco marker, which is placed on the top side of the robot.

## Requirements

- OpenCV v4.2.0 or later with extra modules (aruco)
- [Boost shared memory wrapper](https://github.com/BrOleg5/boost-shared-memory-wrapper) v1.2 or later
- gcc 9.3.0+ or MSVC 19+
- CMake 3.0 or later

## Configure, build and install library and application

```bash
# Create build directory
mkdir build

# Configure as static library
cmake -S mobile-robot-localization/ -B build/

# Build library
cmake --build build/

# Install library
sudo cmake --install build/
```

### Extra options

You can build and install tests:

```bash
# Configure as static library
cmake -S mobile-robot-localization/ -B build/ -D BUILD_TEST=ON
```

If you want to build and install application:

```bash
# Configure application
cmake -S mobile-robot-localization/ -B build/ -D BUILD_APP=ON
```

## Using ArucoLocalization with CMake

Add this strings in your CMakeLists.txt file:

```CMake
find_package(ArucoLocalization 2.0 REQUIRED)
target_link_libraries(<ProjectName> arucolocalization)
# if nessesary, add include directories to target
target_include_directories(<ProjectName> ${ArucoLocalization_INCLUDE_DIRS})
```
