## ProVANT Simulator SDF Parser

The ProVANT Simulator SDF Parser is a ROS package that provides a well tested and documented library for reading the values of Simulation Description Format (SDF) files.

As the documentation from Gazebo is "lacking" in some respects and it is hard to ensure that a method has the desire effect, this library is a wrapper to the SDF parsing utilities provided by Gazebo, and provides methods that are named according to their functions, have good documentation and are covered by unit tests in order to ease the development of Gazebo plugins under the  ProVANT Simulator project.

## Installation

Assuming that you have a working ROS installation in Ubuntu Linux and already cloned the latest ProVANT Simulator git repository, to install this package, simply execute the following commands in your catkin workspace root:

```bash
catkin_make --pkg provant_simulator_sdf_parser
```

## Usage

To use this package, you first need to declare it as a dependency, firstly in the package.xml file of your package:

```xml
...
<depend>provant_simulator_sdf_parser</depend>
...
```

Then it is necessary to find the package as catkin component:
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  provant_simulator_sdf_parser
  ...
)
```

And in the catkin_package setup:
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS
    ...
    provant_simulator_sdf_parser
    ...
  ...
)
```

## Testing 

To run the test cases for this package, run the following command in your catkin workspace root folder:
```bash
catkin_make run_tests_provant_simulator_sdf_parser
```

All of the test suites are based on the Google Test framework and all it is not necessary to download any test data.

## Built with

This project is built with ROS Melodic Morenia, Gazebo 9, and Boost version 1.65.1.


## Features

This library provides methods to:

* Check if an attribute exists in a SDF element;
* Check if an element exists in a SDF element;
* Get the text value of SDF element attributes;
* And reading the following types from SDF elements:
  * Strings
  * Booleans
  * Integers
  * Unsigned Integers
  * Single and double precision floating point numbers.

## Example

To read the values of the following plugin declared in a SDF element:
```xml
<plugin name="test_plugin" filename="test_library.so">
  <enabled>true</enabled>
  <integer_value>10</integer_value>
  <link_name>test_link</link_name>
</plugin>
```

You can use the following C++ code added in your Gazebo plugin load method:
```cpp
#include <provant_simulator_sdf_parser/sdf_parser.h>

void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Check that the SDF element is not null
  GZ_ASSERT(sdf != nullptr, "The plugin received a null SDF element pointer.");
  // Create the SDF Reader element
  SDFParser parser(sdf);

  try{
    bool enabled = parser.GetElementBool("enabled");
    int integer_value = parser.GetElementInt("integer_value");
    std::string link_name = parser.GetElementText("link_name");
  }
  catch(const SDFStatus &e)
  {
    ROS_FATAL_STREAM("An unexpected exception with the following message has ocurred: \"" << e.what() << "\".");
  }
}
```

It is also possible to use the version of the methods that don't throw exceptions:
```cpp
void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  SDFParser parser(sdf);
  int integer_value;
  SDFStatus res = parser.getElementInt("integer_value", &integer_value);
  if(res.isError())
  {
    ROS_FATAL_STREAM("An unexpected error ocurred while reading the \"integer_value\" element. Error message: \"" << res.errorMessage() << "\".");
  }
}
```

## Documentation

The documentation of the classes and methods are generated with rosdoc_lite, and are avaiable under the doc folder after running this command in the source folder of the ProVANT Simulator SDF Parser package:

```bash
rosdoc_lite .
```

## License

This package is part of the ProVANT Simulator project distributed under the terms of the MIT License.
For more details please visit https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md.
