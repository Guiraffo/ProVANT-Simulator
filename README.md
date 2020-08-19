# ProVANT Simulator

ProVANT Simulator is a simulation environment developed in order to validate and evaluate the performance of control strategies. The simulator version addressed in this guide was develop on top of Gazebo, a simulation platform, and ROS (Robot Operating System), a framework for developing robot applications.

ROS offers a programming interface for robotics applications and features repositories with several software modules, and Gazebo is a 3D simulation software under an open source license, maintained and developed by the Open Source Robotics Foundation (OSRF). It is able to simulate the dynamic behavior of rigid, articulated bodies, and includes features such as collision detection and graphical visualization.

## Installation

The following procedures assumed that the target system has a clean, recently installed Ubuntu version 18.04 (Bionic Beaver) operational system.

### Installing Git

To access the source code files for ProVANT hosted on this GitHub repository, Git must be installed on the user's computer. In case it is not, open a new terminal, and run the following commands:

```bash
sudo apt update
sudo apt install git
```

### Installing and configuring ROS

This version of the ProVANT Simulator is developed on top of ROS version Melodic Morenia.

Please install the Desktop Full Install version of ROS according to [this guide](https://wiki.ros.org/melodic/Installation/Ubuntu).

### Installing Qt Framework
To use the ProVANT Simulator GUI (Graphical User Interface), you need to install the Qt Framework version 5.15 LTS.
The online installer for the open-source version of Qt is available [here](https://www.qt.io/download-qt-installer).

After downloading the online installer, open a terminal in the folder where you saved the file, add the execution permission, and run the installer with the following commands:

```bash
chmod +x qt-unified-linux*
./qt-unified-linux*
```

Follow the steps on the graphical installation assistant. Please note that a Qt Account is necessary to complete the installation procedure (this account can be created in the installation assistant).

### Dowloading and installing the simulation environment

The source code for ProVANT Simulator is located in this GitHub repository. To finish the installation process, please clone this repository under the catkin workspace configured during the ROS installation step.

```bash
cd src
git clone https://github.com/Guiraffo/ProVANT-Simulator.git
```

After this step the ProVANT Simulator can be compiled and installed with the following command:

```bash
cd ProVANT-Simulator
chmod +x install.sh
./install.sh
```

Provided that all the procedures described above were executed successfully, the user will be ready to work with ProVANT Simulator.
