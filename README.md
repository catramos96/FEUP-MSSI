# FEUP-MSSI

## ROS

### Run

```sh
# terminal 1
$ cd [path_to_workspace]
$ rm -rf build devel # clean workspace configurations
$ catkin_make 
$ source devel/setup.bash
$ roscore #init master node
```
```sh
# terminal 2
$ cd [path_to_workspace]
$ rosrun turtlesim turtlesim_node #init simulator node
```
```sh
# terminal 3
$ cd [path_to_workspace]
$ source devel/setup.bash
$ rosrun integration turtlebot_teleop.py #init turtlesim controller
```

### Concepts
* Nodes: A node is an executable that uses ROS to communicate with other nodes.
* Messages: ROS data type used when subscribing or publishing to a topic.
* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
* Master: Name service for ROS (i.e. helps nodes find each other)
* Rosout: ROS equivalent of stdout/stderr
* Roscore: Master + rosout + parameter server (parameter server will be introduced later) 

### Comands

#### CREATE/BUILD CATKIN WORKSPACE AND SOURCE FILE
```sh
$ mkdir -p catkin_ws/src  #catkin-_ws: ros workspace
$ cd catkin_ws/
$ catkin_make #build
$ . ~/catkin_ws/devel/setup.bash  #source file
```

#### LOGS
```sh
$ roscd log
```

#### CREATE A CATKIN PACKAGE (inside the src)
```sh
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
#ex:  catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
# ROS settings
$ source <your_workspace_path>/catkin/devel/setup.bash
```

#### MAKE FILES EXECUTABLE
```sh
$ chmod 755 ~/node_example/cfg/node_example_params.cfg #executable
```

#### INIT NODES
```sh
$ roscore #Init master node
$ rosrun [package_name] [node_name] #Init new node
# ex: rosrun turtlesim turtlesim_node
# ex: rosrun turtlesim turtle_teleop_key
```

#### NODES INFORMATION
```sh
$ rosnode list    #List all nodes
$ rosnode info [node_name]    #View info of node rosout
$ rosnode ping [node_name]	  #Test if it's up
$ rosrun rqt_graph rqt_graph	  #View simulation graph
```

### Topics

Nodes subsribe and publish to topics. </br>
Services are like topics but with request/reply. </br>
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29 </br>
