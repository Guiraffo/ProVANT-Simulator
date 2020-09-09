cmake_minimum_required(VERSION 2.8.3)

project(uam3axis)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs tf2 tf2_ros)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
