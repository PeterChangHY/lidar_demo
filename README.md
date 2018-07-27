# lidar demo in Cupertino office



## install

```$git clone git@github.com:PeterChangHY/lidar_demo.git```
```$cd lidar_demo```  
```$catkin_make install```  
```$source devel/setup.bash```

## Config Network  
Config the network of velodyne, follow http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20HDL-32E
1. Power the LIDAR 
2. Connect the LIDAR to an Ethernet port on your computer.
3. Statically assign an IP to this port in the 192.168.3.x range.

## run

```$roslaunch bg_test bg_remove.launch```

