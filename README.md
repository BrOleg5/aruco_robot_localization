# Aruco mobile robot localization

Application and library was created to localize mobile robot by Aruco marker, which is placed on the top side of the robot.

## Requirements

For library:

- OpenCV v4.2.0 or later with extra modules (aruco)
- gcc 9.3.0+ or MSVC 19+
- CMake 3.0 or later

Additionally, for application:

- [Boost shared memory wrapper](https://github.com/BrOleg5/boost-shared-memory-wrapper) v1.2 or later

## Building

You can build this package both pure CMake or colcon.

### CMake

```bash
# Create build directory
mkdir build

# Configure as static library
cmake -S aruco_robot_localization/ -B build/

# Build library
cmake --build build/

# Install library
sudo cmake --install build/
```

### Colcon

Run in your workspace

```bash
colcon build --packages-select aruco_robot_localization
```

### Extra options

You can build and install tests:

```bash
# CMake
cmake -S aruco_robot_localization/ -B build/ -DBUILD_TEST=ON

# Colcon
colcon build --packages-select aruco_robot_localization --cmake-args -DBUILD_TEST=ON
```

If you want to build and install applications:

```bash
# CMake
cmake -S aruco_robot_localization/ -B build/ -DBUILD_APP=ON

# Colcon
colcon build --packages-select aruco_robot_localization --cmake-args -DBUILD_APP=ON
```

## Usage from an external CMake project

Add this lines in your CMakeLists.txt file:

```CMake
find_package(aruco_robot_localization REQUIRED)
target_link_libraries(YOU_TARGET_NAME ${aruco_robot_localization_LIBS})
target_include_directories(YOU_TARGET_NAME ${aruco_robot_localization_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11
```
