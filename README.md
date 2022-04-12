# ProVANT Simulator

ProVANT Simulator is a simulation environment developed in order to validate and evaluate the performance of control strategies. The simulator version addressed in this guide was develop on top of Gazebo, a simulation platform, and ROS (Robot Operating System), a framework for developing robot applications.

ROS offers a programming interface for robotics applications and features repositories with several software modules, and Gazebo is a 3D simulation software under an open source license, maintained and developed by the Open Source Robotics Foundation (OSRF). It is able to simulate the dynamic behavior of rigid, articulated bodies, and includes features such as collision detection and graphical visualization.

## Installation

The following procedures assumed that the target system has a clean, recently installed Ubuntu version 20.04.4 (Focal Fossa) operational system.

### Update Ubuntu

As this tutorial assumes that a fresh Ubuntu installation is available, it is important to ensure that you have update versions of all packages, to do this:

1. Open a terminal application
2. Execute the following command

```bash
sudo apt update
```

Once the above command is finished, update all packages in your distribution using this command:

```bash
sudo apt dist-upgrade --yes
```

### Installing CMake

The version of CMake that is available by default in Ubuntu is very old, and do not offer several of the features required by the ProVANT Simulator, therefore it is necessary to install a newer version of cmake.

To do this, please open an ew terminal and execute the following steps

1. Make sure that the gpg and wget packages are installed, by running the following command

```bash
sudo apt install --yes gpg wget 
```

2. Add the signing key of the kitware cmake repository

```bash
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
```

3. Add the repository to apt sources

```bash
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
```

4. Update the apt package database

```bash
sudo apt update
```

5. Remove the signing key used to install the repository, as it is no longer needed

```bash
sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg
```

6. Install kitware keyring, to ensure you are able to update cmake when future versions are released

```bash
sudo apt install --yes kitware-archive-keyring
```

7. Update cmake

```bash
sudo apt install cmake
```

8. Verify that you have an updated version, by running the following command

```bash
cmake --version
```

The output of the above command should be something like:

```
cmake version 3.23.1
```

A version of CMake newer than 3.21 is required to compile the simulator.

### Install Qt

The ProVANT Simulator GUI is built using the Qt framework. To compile the GUI, please execute the following step in a terminal

```bash
sudo apt install --yes build-essential qtcreator qt5-default qtchooser libqt5serialport5-dev
```

### Installing ROS

Once a suitable version of CMake and Qt are installed, the next step is to install ROS Noetic, by opening a terminal and executing the following steps:

1. Add the ROS repository to apt sources

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2. Ensure that the curl package is installed

```bash
sudo apt install --yes curl
```

3. Install ROS signing key

```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

4. Update the apt package database

```bash
sudo apt update
```

5. Install the Desktop Full version of ROS Noetic:

```bash
sudo apt install --yes ros-noetic-desktop-full
```

6. After ROS is installed, it is necessary to add it to your bashrc script so the ROS environment variables are available in every new terminal opened:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

7. Source the ROS configuration script

```bash
source /opt/ros/noetic/setup.bash
```

8. Install the software required for compiling ROS packages

```bash
sudo apt install --yes python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential libmsgsl-dev
```

9. Ensure that the rosdep tool is installed

```bash
sudo apt install python3-rosdep
```

10. Initialize rosdep

```bash
sudo rosdep init
```

11. And update the rosdep release database

```bash
rosdep update
```

12. Finally, close your terminal, and open a new one. To verify that ROS was installed and configured correctly, please execute the following command:

```bash
printenv | grep ROS
```

The output of the above command should be similar to this

```bash
ROS_VERSION=1
ROS_PYTHON_VERSION=3
ROS_PACKAGE_PATH=/opt/ros/noetic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ROS_MASTER_URI=http://localhost:11311
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_DISTRO=noetic
```

### Create a Catkin Workspace

To build the ProVANT Simulator, a catkin workspace is required. To create such workspace, please open a terminal and execute the following sequence of commands:

1. Crate a folder for your workspace

```bash
mkdir -p ~/catkin_ws/src
```

2. Open the folder created in the previous step

```bash
cd ~/catkin_ws
```

3. Execute catkin_make for the first time to setup your workspace

```bash
catkin_make
```

4. Add the workspace to your .bashrc file to ensure it will be available in every new terminal

```bash
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

5. To verify that catkin workspace was configured correctly, close your terminal, open a new one, and run the following command:

```bash
echo $ROS_PACKAGE_PATH
```

The output of this command should look similar to this

```bash
/home/user/catkin_ws/src:/opt/ros/noetic/share
```

The output must contain the package of the ROS noetic installation (/opt/ros/noetic/share) and the path to your catkin workspace (/home/user/catkin_ws/src) where "user" is the name of your linux user.

### Installing Git

To access the source code files for ProVANT hosted on this GitHub repository, Git must be installed on the user's computer. In case it is not, open a new terminal, and run the following commands:

```bash
sudo apt --yes install git
```

### Dowloading the simulation environment

The source code for ProVANT Simulator is located in this GitHub repository. To finish the installation process, please clone this repository under your catkin workspace by running the following commands:

1. Open the catkin workspace source folder

```bash
cd $HOME/catkin_ws/src
```

2. Clone the ProVANT Simulator git repository

```bash
git clone https://github.com/Guiraffo/ProVANT-Simulator.git
```

### Install Conan

The ProVANT Simulator uses the Conan package manager to install a few packages that are not available in the Ubuntu software repositories.

To install Conan, please open a terminal and execute the following steps:

1. Install the pip package

```bash
sudo apt install --yes python3-pip
```

2. Install conan

```bash
sudo pip3 install --upgrade conan
```

3. Execute the following command to check that conan was installed

```bash
conan --version
```

The output of this command should be similar to

```bash
Conan version 1.47.0
```

### Add the ProVANT Simulator Environment Variables

The ProVANT Simulator requires a series of environment variables in order to work correctly. To create the variables, first, execute the following command:

```bash
gedit ~/.bashrc
```

A new window of the gedit text editor will open, with the contents of your .bashrc file.

Go to the end of this file, and paste the following lines:

```bash
# ProVANT Simulator Environment Variables
export TILT_PROJECT=$HOME/catkin_ws/src/ProVANT-Simulator/
export PROVANT_ROS=$HOME/catkin_ws/src/
export DIR_ROS=$HOME/catkin_ws/
export TILT_STRATEGIES=$HOME/catkin_ws/devel/lib/
export TILT_MATLAB=$HOME/catkin_ws/src/ProVANT-Simulator/source/Structure/Matlab/
export PROVANT_DATABASE=$HOME/catkin_ws/src/ProVANT-Simulator/source/Database/
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/ProVANT-Simulator/source/Database/models/
```

Save the file in gedit, and close your terminal.

### Compile the ProVANT Simulator ROS Packages

To compile the ProVANT Simulator ROS packages, open a new terminal, and execute the following sequence of commands:

1. Open your catkin workspace folder

```bash
cd ~/catkin_ws
```

2. Execute catkin_make to compile the packages

```bash
catkin_make
```

After this step the ProVANT Simulator can be compiled and installed with the following command:

```bash
cd ProVANT-Simulator
chmod +x install.sh
./install.sh
```

**Note:** If the catkin_make command fails without an explicit compilation error, try running it again.

## Compile and Install the ProVANT Simulator GUI

The ProVANT Simulator includes a Graphical User Interface (GUI) that allows the user to use the simulator with depending solely on the command line.

To compile and install the GUI, please open a terminal and execute the following sequence of commands:

1. Open the ProVANT Simulator source folder

```bash
cd $HOME/catkin_ws/src/ProVANT-Simulator/source
```

2. Create a build folder for the GUI

```bash
mkdir build
```

3. Open the GUI build folder

```bash
cd build
```

4. Configure the GUI compilation

```bash
qtchooser -qt=5 -run-tool=qmake ../GUI/GUI.pro -r -spec linux-g++
```

5. Compile the GUI

```bash
make
```

6. Install the GUI

```bash
sudo ln -sf $HOME/catkin_ws/src/ProVANT-Simulator/source/build/GUI /usr/local/bin/provant_gui
```

7. Check that the GUI is opening correctly

```bash
provant_gui
```

If a new window with the ProVANT Simulator GUI opens, the compilation and installation was successful.

### Use the Simulator

Provided that all the procedures described above were executed successfully, the user will be ready to work with ProVANT Simulator.
