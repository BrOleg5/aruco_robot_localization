# Aruco mobile robot localization

Application and library was created to localize mobile robot by aruco marker, which is placed on the top side of the robot.

## Requirements

- OpenCV 4.20 or later with extra modules (aruco)
- Boost library v1.77.0 or later
- [Boost shared memory wrapper](https://github.com/BrOleg5/boost-shared-memory-wrapper) (already included in the repository)

## Building and install application and library

### Configure aruco-robot-localization library as static library

```
# Create build directory
mkdir build

# Configure
cmake -S mobile-robot-localization/ -B build/ -DCMAKE_INSTALL_PREFIX="<path_to_install>"
```

### Configure the library as shared library

```
# Create build directory
mkdir build

# Configure
cmake -S mobile-robot-localization/ -B build/ -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX="<path_to_install>"
```

### Build the application and the library

```
# Build
cmake --build build/
```

### Install aruco-robot-localization library

```
cmake --install build/
```

Also you can build and install library at the same time:
```
cmake --build build/ --target install
```

## Usage application

```
usage: ./localization [options]

Options:

  --help         Display this information.
  -cam <index>   Use webcamera with <index> in system.
  -t <duration>  Set <duration> of program execution in ms.
  -shared-memory Use shared memory from boost to transfer measurements.
```