import os
import time

os.system("gnome-terminal -e 'bash -c \"cd COM_CENTER;python runner.py '127.0.0.1' 5004 0.1 10 5;cd ..;exec bash\"'")
os.system("gnome-terminal -e 'bash -c \"cd ROS; rm -rf build devel;catkin_make; source devel/setup.bash;exec bash\"'")
time.sleep(4)
os.system("gnome-terminal -e 'bash -c \"cd ROS; roscore;exec bash\"'")
time.sleep(1)
os.system("gnome-terminal -e 'bash -c \"cd ROS; rosrun turtlesim turtlesim_node;exec bash\"'")
os.system("gnome-terminal -e 'bash -c \"cd ROS; source devel/setup.bash;rosrun integration turtlebot_teleop.py '127.0.0.1' 5004 '127.0.0.1' 3003 0.5 1.0 1 0.1;exec bash\"'")
