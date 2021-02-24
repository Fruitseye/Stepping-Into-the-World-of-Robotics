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
* ROS Topics allow unidirectional communication between ROS nodes. When using ROS Topics, a ROS node can be a publisher, subscriber or both.
* Publisher Node – publishes data on a ROS Topic and a Subscriber Node – subscribes to a ROS Topic to get data. 
* So, Publisher and Subscriber Nodes exchange ROS messages over ROS Topics
* ROS Message – a simple data structure which can hold data of various types (int, float, bool, etc.)
```
rostopic
```
- displays information about ROS Topics, publishers, subscribers, publishing rate and ROS Messages.
```
$ rostopic list – shows a list of all topics currently subscribed to and published.
$ rostopic echo [topic] – shows the data published on a topic. 
$ rostopic type [topic] – returns the message type of any topic being published.
$ rostopic pub [topic] [msg_type] [args] – publishes data on a topic.
```

### Understanding ROS Topics using TurtleSim
* Make sure ROS Master is running:
```
$ roscore
```
* In a new terminal, run the TurtleSim Node:
```
$ rosrun turtlesim turtlesim_node
```
* In another terminal, run:
```
$ rosrun turtlesim turtle_teleop_key
```
* Now, move the turtle with the arrow keys on your keyboard. Think about what's happening here!  
* rqt-graph: Creates a dynamic graph of what’s going on in the system.      
* rqt_graph is a part of the rqt package. To install it, run:   
```
$ sudo apt-get install ros-<distro>-rqt        
$ sudo apt-get install ros-<distro>-rqt-common-plugins    
```
* In a new terminal, run:   
```
$ rosrun rqt_graph rqt_graph 
```
* rostopic pub – publishes data on a topic currently advertised. Try:
```
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
* Also try:
```
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```

##### Note: Refer to PPT for information on some important ROS Terminology

### Launch two ROS Nodes     
Let’s see how to use the roslaunch command to launch two nodes, namely talker and listener nodes.
* Create a chatter.launch file and save in a folder named launch, in the pkg_ros_basics package  
```
$ roscd pkg_ros_basics
$ mkdir launch 
$ cd launch 
$ touch chatter.launch
$ gedit chatter.launch
```
* Copy and Paste the following code:
```
<launch>
  <node name="talker" pkg="pkg_ros_basics" type="talker.py" output="screen"/>
  <node name="listener" pkg="pkg_ros_basics" type="listener.py" output="screen"/>
</launch>
```
* Save and Exit
* Change to scripts folder and create two files - talker.py and listener.py:
```
$ cd ..
$ cd scripts
$ touch talker.py listener.py
$ gedit talker.py listener.py 
```
* Copy and paste the following codes into their respective files:     

**talker.py
```
#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
**listener.py
```
#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
* Save and exit
* Make these files executables:
```
$ sudo chmod +x talker.py
$ sudo chmod +x listener.py
```
* Now, to launch these two nodes, run:
```
$ roslaunch pkg_ros_basics chatter.launch
```




