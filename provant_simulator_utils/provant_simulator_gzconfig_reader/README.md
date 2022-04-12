## ProVANT Simulator GZConfig Reader

The ProVANT Simulator Gazebo (GZ) Config Reader is a package that provides a static library that allow user to obtain an instance of the ConfigReader without any run time ROS dependency.

## Installation

Assuming that you have a working ROS installation in Ubuntu Linux and already cloned the latest ProVANT Simulator git repository, to install this package, simply execute the following commands in your catkin workspace root:

```bash
catkin_make --pkg provant_simulator_gzconfig_reader
```

## Usage

To use this package, you first need to declare it as a dependency, firstly in the package.xml file of your package:

```xml
...
<depend>provant_simulator_gzconfig_reader</depend>
...
```

Then it is necessary to find the package as catkin component:
```cmake
find_package(catkin REQUIRED COMPONENTS
  ...
  provant_simulator_gzconfig_reader
  ...
)
```

And in the catkin_package setup:
```cmake
catkin_package(
  ...
  CATKIN_DEPENDS
    ...
    provant_simulator_gzconfig_reader
    ...
  ...
)
```

## Built with

This project is built with ROS Melodic Morenia, Gazebo 9, and Boost version 1.65.1.

## Example

The following example shows the process of obtaining an instance of the ConfigReader class and printing the name of the control strategy of the current simulation:
```cpp
#include <iostream>
#include <memory>

#include <provant_simulator_gzconfig_reader/gzconfig_reader.h>

void Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  GazeboConfigReader gzReader(world->Name());
  std::unique_ptr<ConfigReader> reader(gzReader.GetConfigReader());
  if (reader)
  {
    const auto val = reader->getControlStrategy();
    std::cout << "Control strategy name: " << val << " \n";
  }
  else
  {
    std:cerr << "An error ocurred while loading the ConfigReader instance.\n";
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
