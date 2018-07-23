power up lidar/laptop , connect to velodyne, change network to : 192.168.3.x, subnet: 255.255.255.0

1. open terminal #1 (velodyne)

$ roslaunch velodyne_pointcloud run_64e_s3.launch rpm:=600 max_range:=10 min_range:=1.5

2. open terminal #2 (background substraction)

$ rosrun bg_test bg_sub input:=/velodyne_points

4. open terminal #4 (open rviz), you can open a conf file in conf/rviz_conf.rviz

$ rviz -f velodyne


Enjoy!
