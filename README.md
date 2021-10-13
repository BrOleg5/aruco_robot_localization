# Aruco mobile robot localization

Application and library was created to localize mobile robot by aruco marker, which is placed on the top side of the robot.

## Requirements

- OpenCV v4.20 or later with extra modules (aruco)
- [Boost shared memory wrapper](https://github.com/BrOleg5/boost-shared-memory-wrapper) v1.1 or later

## Configure, build and install library and application

```
# Create build directory
mkdir build

# Configure as static library
cmake -S mobile-robot-localization/ -B build/

# Build library
cmake --build build/

# Install library
sudo cmake --install build/
```

You can also build and install shared library:
```
# Configure as shared library
cmake -S mobile-robot-localization/ -B build/ -DBUILD_SHARED_LIBS=ON
```

You can build and install tests:
```
# Configure as static library
cmake -S mobile-robot-localization/ -B build/ -DBUILD_TESTS=ON -DINSTALL_TESTS=ON
```

If you want to build and install application, 
```
# Configure shared library and application
cmake -S mobile-robot-localization/ -B build/ -DBUILD_SHARED_LIBS=ON -DBUILD_APP=ON -DINSTALL_APP=ON
```

## Using ArucoLocalization with gcc and CMake

Add this strings in your CMakeLists.txt file:
```
find_package(ArucoLocalization 1.2 REQUIRED)
target_link_libraries(<ProjectName> ArucoLocalizationLib)
# if nessesary, add include directories to target
target_include_directories(<ProjectName> ${ArucoLocalization_INCLUDE_DIRS})
```

## Usage application

```
usage: localization [options]

Options:

  --help         Display this information.
  -cam <index>   Use webcamera with <index> in system.
  -t <duration>  Set <duration> of program execution in ms.
  -shared-memory Use shared memory from boost to transfer measurements.
```