# Aruco mobile robot localization

Application and library was created to localize mobile robot by aruco marker, which is placed on the top side of the robot.

## Dependencies

OpenCV 4.20 or later with extra modules (aruco).

Boost library v1.77.0 or later.

## Usage application

```
usage: localization [options]

Options:

  --help         Display this information.
  -cam <index>   Use webcamera with <index> in system.
  -t <duration>  Set <duration> of program execution in ms.
  -shared-memory Use shared memory from boost to transfer measurements.
```