# ProVANT Simulator

ProVANT Simulator is a simulation environment developed in order to validate and evaluate the performance of control strategies. The simulator version addressed in this guide was develop on top of Gazebo 7, a simulation platform, and ROS (Robot Operating System), a framework for developing robot applications, under the Kinetic distribution. In order to use it, a computer running the operating system Ubuntu 16.04 is needed.

ROS offers a programming interface for robotics applications and features repositories with several software modules, and Gazebo is a 3D simulation software under free license, maintained and developed under responsibility of the Open Source Robotics Foundation (OSRF). It's able to simulate the dynamic behavior of rigid, articulated bodies, and includes features such as collision detection and graphical visualization.

## Installation

For the following procedures, it is assumed that the computer in which the simulator is being installed runs a clean, recently installed copy of [Ubuntu 16.04].

### Installing Git

In order to access the source code flies for ProVANT hosted on this GitHub repository, [Git] must be installed in the user's computer. In case it's not, open a new terminal and run the following commands:
```
sudo apt update
sudo apt install git
```

### Installing and configuring ROS

ROS offers a programming interface for robotics applications and several software modules, among which is simulator Gazebo, version 7, used by ProVANT Simulator. Below are the instructions needed for installing the ROS Kinetic Kame distribution (more details and help [here][ROS Wiki]).

#### Configure the Ubuntu repositories
Before starting installation, the Ubuntu repositories must be configured to allow **restricted**, **universe** and **multiverse**.  
*Note*: Usually, this options are already set upon installation of Ubuntu 16.04. In case they're not, follow the [Ubuntu guide] for instructions on doing this.

#### Setup *sources.list*
The computer must also be set up to accept software from *packages.ros.org*. To do that, open a new terminal and run the following command:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Setup access keys for ROS repositories
Run the following command:
```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

#### Installation
First, make sure the Debian package index is up-to-date:
```
sudo apt update
```
After updating the packages, download the binary files:
```
sudo apt install ros-kinetic-desktop-full
```

#### Initialize *rosdep*
Before using ROS, **rosdep** must be initialized. This system installs the system dependencies for the source code to be compiled. It is required in order to run some core components in ROS.
```
sudo rosdep init
rosdep update
```

#### Environment setup
To have the ROS environment variables automatically added to the bash session every time a new shell is launched:
```bash
echo "source /opt/ros/kinetic/setup.bash" >> $HOME/.bashrc
source ~/.bashrc
```

#### Create a ROS workspace
Finally, a ROS workspace must be created. To do that, run the following command sequence:
```bash
mkdir -p $HOME/catkin_ws/src
cd $HOME/catkin_ws/
catkin_make
source $HOME/catkin_ws/devel/setup.bash
echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
```

### Installing Qt Framework
In order to appropriately install ProVANT Simulator's graphical setup environment, QtCreator 5 IDE must be installed. The installation package can be downloaded [here][Qt Creator].

### Dowloading and installing the simulation environment
The [source code] for ProVANT Simulator is located in this GitHub repository. It must be cloned into the user's computer:
```bash
cd $HOME/catkin_ws/src
git clone https://github.com/Guiraffo/ProVANT-Simulator.git
```
Once cloned, the simulation environment can be installed and configured by running the following commands:
```bash
cd $HOME/catkin_ws/src/ProVANT-Simulator
sudo chmod +x install.sh
./install.sh
```
Provided that all the procedures described above were executed successfully, the user will be ready to work with ProVANT Simulator.


[Git]: https://git-scm.com
[Qt Creator]: https://www.qt.io/download-open-source/?hsCtaTracking=f977210e-de67-475f-a32b-65cec207fd03%7Cd62710cd-e1db-46aa-8d4d-2f1c1ffdacea
[ROS Wiki]: https://wiki.ros.org
[Source code]: https://github.com/Guiraffo/ProVANT-Simulator
[Ubuntu 16.04]: http://releases.ubuntu.com/16.04/
[Ubuntu guide]: https://help.ubuntu.com/community/Repositories/Ubuntu
