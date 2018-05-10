power up lidar/laptop 

1. open terminal #1 (velodyne)

$ roslaunch velodyne_pointcloud run_64e_s3.launch rpm:=1000 max_range:=6 min_range:=1.5

2. open terminal #2 (background substraction)

$ rosrun bg_test bg_sub input:=/velodyne_points

3. open terminal #3 (start record background)
 it will pub a string msg to background substraction node to start the background substraction
 so before you type this commandline, hide yourself or keep the space clear.

$ rostopic pub update_bg std_msgs/String hello -1

4. open terminal #4 (open rviz)

$ rviz -f velodyne

Enjoy!
