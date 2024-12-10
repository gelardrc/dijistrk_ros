# dijistrk_ros

A package that implements dijstrik algorithm to find minimum path in occupancy grid map in ROS.

# How to install

1. Open a terminal and start roscore

> roscore

2. Open another terminal and start map_server

> cd <dijkstra_ros>/map
> rosrun map_server map_server example.yaml

3. Open another terminal

> rosrun dijkstra_ros dijkstra_ros.py <_choose> <_start> <_goal>

🚀️ Here you set _choose parameter to true if you want to pick start and goal points inside rviz
🚀️ Or you can set  _start and _goal parameters

4. Finally open another terminal

> cd <dijkstra_ros>/rviz
> rosrun rviz rviz -d example.rviz

5. (optional) If you set _choose parameter to true, click on 2D pose estimate pose to set start point and click on 2D nav goal to set goal point. You can change them whenever you want in order to recalculate path.

# Example :

> roslaunch dijkstra_ros example.launch choose:="true"

Now usgin rviz tou can send start and goal points like this : 

![](https://github.com/gelardrc/dijkstra_ros/blob/main/img/dijstrik_ros.gif)



# To do list

* [ ] Add inflation Raddius on map (working)


