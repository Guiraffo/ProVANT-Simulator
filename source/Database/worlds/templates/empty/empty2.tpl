<?xml version="1.0" encoding="UTF-8"?>
<!--
This file is part of the ProVANT simulator project.
Licensed under the terms of the MIT open source license. More details at
https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
-->
<sdf version="1.6">
  <world name="empty2.tpl">
    <gui>
      <plugin name="provant_gui_plugin" filename="libprovant_auto_stepping_plugin.so"/>
    </gui>
    <gravity>0.000000 0.000000 -9.8000</gravity>
    <physics type="ode">
      <max_step_size>0.001000</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin name="gazebo_tutorials" filename="libgazebo_ros_world_plugin.so"/>
    <include>
      <uri>model://ground_plane</uri>
      <static>true</static>
    </include>
    <include>
      <uri>model://sun</uri>
      <static>true</static>
    </include>
    <include>
      <uri>model://vant_2.0</uri>
      <name>newmodel</name>
      <static>false</static>
      <pose>0 0 2 0 0 0</pose>
    </include>
    <scene>
      <sky>
        <time>18</time>
        <clouds>
          <speed>0</speed>
        </clouds>
      </sky>
    </scene>
  </world>
</sdf>
