# Basics of ROS
### Setting up ROS Environment
During the installation of ROS, you will see that you are prompted to source one of several setup.*sh files, or even add this 'sourcing' to your shell startup script. This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup.    
If you just installed ROS from apt on Ubuntu then you will have setup.*sh files in '/opt/ros/<distro>/', and you could source them like so: 
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

