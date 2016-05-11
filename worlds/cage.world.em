<?xml version="1.0" ?>
<sdf version="1.4">
<world name="cage">
<gui>
  <camera name="camera">
    <pose>3 -2 3.5 0.0 .85 2.4</pose>
    <view_controller>orbit</view_controller>
  </camera>
</gui>
<include><uri>model://sun</uri></include>
<include><uri>model://ground_plane</uri></include>
    <include>
      <name>fiducial</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>model://ground_fiducials</uri>
    </include>
</world>
</sdf>
