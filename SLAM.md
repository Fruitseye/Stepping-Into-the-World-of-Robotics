# Trying and getting a feel of SLAM(Simulataneous Localistaion and mapping) used on a robot

#### Robot to be used: QUADRUPED ROBOT NAMED CHAMP

## 1) Installation

Rosdep is a command-line tool for installing system dependencies
```
sudo apt install -y python-rosdep
```
Go to src folder of your workspace that you created
```
cd <your_ws>/src
```
Clone the given github repos

```
git clone --recursive https://github.com/chvmp/champ
git clone https://github.com/chvmp/champ_teleop
```
Now Installing the necessary dependencies using rosdep
```
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

**NOTE: After installing any new dependencies or changed your code, you need to run the following**
```
cd <your_ws>
catkin_make
source devel/setup.bash
```
## 2)Walking example
```
roslaunch champ_config gazebo.launch
```
Once it is launched ,launch the keyboard control launch file,which is

```
roslaunch champ_teleop teleop.launch
```

## 3) SLAM using the robot

Start the gazebo environment
```
roslaunch champ_config gazebo.launch 
```

Then type the given code to implement SLAM
```
roslaunch champ_config slam.launch rviz:=true
```

To start mapping:

 - Click '2D Nav Goal'.

 - Click and drag at the position you want the robot to go.

 - Once it reaches there, repeat the above two steps for a different location and direction 
 
 - *Note: Sometimes the bot will not be able to reach a given location and produce "path failed" error,just close both gazebo and rviz and start from first*
 
 - Do this till entire area is covered
 
 - Once done we need to save the map mapped by the bot
 
 ```
 roscd champ_config/maps
 rosrun map_server map_saver
 ```
 
 To view the map:
   Go to **champ/champs_config/maps/** and open map.pgm and voila!!
