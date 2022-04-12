## ProVANT Simulator SDF Reader

The ProVANT Simulator Math Utils is package that provides mathematical objects commonly used on the development of control strategies, such as numerical integrators and differentiators.

As there are several elements commonly used in control design, this library provides a well documented and tested collection of elements to facilitate the creation of new control strategie and reduce code duplication.

## Installation

Assuming that you have a working ROS installation in Ubuntu Linux and already cloned the latest ProVANT Simulator git repository, to install this package, simply execute the following commands in your catkin wokspace root:

```bash
catkin_make --pkg provant_simulator_math_utils
```

## Usage

To use this package, you first need to include declare it as a dependency, firstly in the package.xml file of your package:

```xml
...
<depend>provant_simulator_math_utils</depend>
...
```

Then it is necessary to find the package as catkin component:
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  provant_simulator_math_utils
  ...
)
```

And in the catkin_package setup:
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS
    ...
    provant_simulator_math_utils
    ...
  ...
)
```

## Testing 

To run the test cases for this package, run the following command in your catkin workspace root folder:
```bash
catkin_make run_tests_provant_simulator_math_utils
```

All of the test suites are based on the Google Test framework and all it is not necessary to download any test data.

## Built with

This project is built with ROS Melodic Morenia, and Eigen3.

## Features

Currently this library provides a header only implementation of a templated integrator based on the trapezoidal rule.

## Example

The following example returns the integral of a the function f(x) = x along the interval from 0 to 5, evaluted on a grid with a constant step of 1:
```cpp
#include <provant_simulator_math_utils/integrator.h>

double integrate()
{
  Integrador<double> integrator(1.0);

  integrator.update(1.0);
  integrator.update(2.0);
  integrator.update(3.0);
  integrator.update(4.0);
  integrator.update(5.0);

  return integrator.value();
}
```

## Documentation

The documentation of the classes and methods are generated with rosdoc_lite, and are avaiable under the doc folder after running this command in the source folder of the ProVANT Simulator SDF Reader package:

```bash
rosdoc_lite .
```

## License

This package is part of the ProVANT Simulator project distributed under the terms of the MIT License.
For more details please visit https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md.
