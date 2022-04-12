## ProVANT Simulator Parser Utils

The ProVANT Simulator Parser Utils is package that provides methods to parse basic types from strings.

These functions can be used for example to parse values from a XML config file or from the SDF description of a plugin.

## Installation

Assuming that you have a working ROS installation in Ubuntu Linux and already cloned the latest ProVANT Simulator git repository, to install this package, simply execute the following commands in your catkin wokspace root:

```bash
catkin_make --pkg provant_simulator_parser_utils
```

## Usage

To use this package, you first need to include declare it as a dependency, firstly in the package.xml file of your package:

```xml
...
<depend>provant_simulator_parser_utils</depend>
...
```

Then it is necessary to find the package as catkin component:
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  provant_simulator_parser_utils
  ...
)
```

And in the catkin_package setup:
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS
    ...
    provant_simulator_parser_utils
    ...
  ...
)
```

## Testing 

To run the test cases for this package, run the following command in your catkin workspace root folder:
```bash
catkin_make run_tests_provant_simulator_parser_utils
```

All of the test suites are based on the Google Test framework and all it is not necessary to download any test data.

## Built with

This project is built with ROS Melodic Morenia, and Boost 1.63.1.

## Features

This library provides methods to parse the following types from strings:

* boolean
* int
* long int
* unsigned int
* float
* double
* long double

## Example

The following example shows the conversion from a string to a few numeric values
and a boolean.
```cpp
#include <provant_simulator_parser_utils/type_conversion.h>

void convert()
{
  int intVal = ParseInt("10");
  int negVal = ParseInt("-10");
  float = ParseInt("-10.0");
  double pi = ParseDouble("3.141592");
  bool enabled = ParseBool("true");
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
