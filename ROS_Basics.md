# Basics of ROS
### Setting up ROS Environment
During the installation of ROS, you will see that you are prompted to source one of several setup.* *sh files*, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup.    
If you just installed ROS from apt on Ubuntu then you will have setup.* *sh files* in '/opt/ros/<distro>/', and you could source them like so: 
 ```
$ source /opt/ros/<distro>/setup.bash
  ```
  So, in our case:
  ```
$ source /opt/ros/noetic/setup.bash
  ```     
You will need to run this command on every new shell you open to have access to the ROS commands, unless you add this line to your .bashrc. To add this to your .bashrc, do the following:
 ```
 $ nano ~/.bashrc
 ```     
 This will open up .bashrc. Go to the bottom and add the text:
 ```
 source /opt/ros/noetic/setup.bash
 ```      
 Save and Exit
 
 ### Creating a ROS Workspace    
 * A ROS workspace is a folder where you modify, build and install ROS packages.
 * catkin is the official build system of ROS.      
To create a catkin workspace:	    
```
$ mkdir –p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
If you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.* *sh files*. Sourcing any of these files will overlay this workspace on top of your environment. Before continuing source your new setup.* *sh file*:  
```
$ source devel/setup.bash
```
[Reference](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)    

### Creating a catkin package
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg pkg_ros_basics std_msgs rospy roscpp
```    
* This will create a pkg_ros_basics folder containing a package.xml and a CMakeLists.txt.      
```
$ cd ~/catkin_ws 
$ catkin_make
```
### ROS Master
* The role of the ROS Master is to enable individual ROS nodes to locate one another, so that they can communicate with each other peer-to-peer.      
* So, communication between ROS Nodes is established by ROS Master. 
* To start ROS Master, run:
```
$ roscore
```
### Create a ROS Node
* Make sure you are in ~/catkin_ws and your setup files are all sourced.     
* Navigate to pkg_ros_basics
```
roscd pkg_ros_basics
```
* If roscd didn't work, it is maybe because you have not sourced the setup files. So, in ~/catkin_ws run:
```
$ source devel/setup.bash
```     
* In the pkg_ros_basics package, create a folder for your python scripts:
```
$ mkdir scripts
$ cd scripts
```
```
$ touch node_hello_ros.py
$ gedit node_hello_ros.py
```
* Now copy and paste the following code in node_hello_ros.py:    
```
#!/usr/bin/env python3

import rospy


def main():

    # 1. Make the script a ROS Node.
    rospy.init_node('node_hello_ros', anonymous=True)

    # 2. Print info on console.
    rospy.loginfo("Hello World!")

    # 3. Keep the node alive till it is killed by the user.
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

```
* Save and Exit
* To make this file an executable, run: 
``` 
$ sudo chmod +x node_hello_ros.py  
```
* While running any executable in ROS, ROS Master has to be running. ROS Master, as the name suggests - links together everything.
* So, while running a ROS Node also, ROS Master should be running!
* To run the node we created: 
```
$ rosrun pkg_ros_basics node_hello_ros.py
```

### ROS Topics


