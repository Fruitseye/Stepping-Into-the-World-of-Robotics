# Introduction to URDF and building a two wheeled robot from scratch

 - The syntax will be similar to html syntax since XML is a markup language similar to html
 - We will write a .xacro file which stands for "XML macro" language instead of a .urdf file. Reason :Think Why?( ~Google why~)
I

### Some of the Tags that we will be using in our code:

```
<robot>
<material>
<color>
<gazebo>
<sensor>
<inertia>
<link>
<joint>
<collision>
..... and many more .......
```

### Lets dive into the code

 **1)Include \<xml\> and \<robot\> tag**
```
<?xml version="1.0" ?>
<robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
</robot>
```

 **1)Create a chassis for the robot**
