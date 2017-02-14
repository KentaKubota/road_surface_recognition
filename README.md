# road_surface_recognition pkg

This ROS package provides the method making occupancy grid map that express lawn place data as occupancy.
For example, if you use the map (Fig. 1), you can make the following map (Fig. 2). 

## Images
![top-page](https://raw.githubusercontent.com/road_surface_recognition/images/map.png)

![new-vm-page](https://raw.githubusercontent.com/road_surface_recognition/images/mapFileName.png)


## Preparation

Please install 'laser_geometry' pkg in your catkin_ws/src/ because this pkg uses laser_geometry pkg function.
<https://github.com/ros-perception/laser_geometry.git>

## How to use


1. $ roslaunch road_surface_recognition build_reflection_mapping.launch

2. $ rosrun map_server map_server ~/catkin_ws/src/road_surface_recognition/test/map.yaml


 
3. $ unzip ~/catkin_ws/src/road_surface_recognition/test/bagFile.zip
   $ rosbag play ~/catkin_ws/src/road_surface_recognition/test/bagFile.bag

    Run following command when bagfile is finished.
4. $ rosservice call /up_map 


    if you can see following message on your terminal, 'Published an OccupancyGrid data of which topic name is occupancyGrid', finally, run following command. 
 
5. $ rosrun map_server map_saver -f mapFileName map:=lawnOccupancyGrid
 
     you can get occupancy grid map named mapFileName on terminal having done command.
















