# Exercise 3
* When running exercises don't forget to do source devel/setup.bash
## Exercise 3a: Teleoperate the pan and tilt degrees of freedom by using the arrow keys
* The pan can be teleoperated by arrow keys controlled by the turtlesim/teleop_turtle node.
* Exercise 3a can be launched with following command: roslaunch urdf_tutorial urdf_tutorial_a.launch
## Exercise 3b
* In this extension of exercise you can change the scale of teleoperating.
* Exercise 3b can be launched with following command: roslaunch urdf_tutorial urdf_tutorial_b.launch
* The scale can be changed with rosservice call /set_scale 10 (you can replace number 10 with desired scaling factor)
## Exercise 3c
* In this exercise you can teleoperate joints 0 and 3 of 7 DOF robot 
* Exercise 3c can be launched with following command: roslaunch urdf_tutorial urdf_tutorial_arm.launch
## Exercise 3d
* In this extension of exercise you can choose the joints being operated.
* Exercise 3d can be launched with following command: roslaunch urdf_tutorial urdf_tutorial_arm2.launch
* The joints operated can be changed with rosservice call /operate_joints 3 5 (you can replace numbers 3 and 5 with joints you want to teleoperate)
## Exercise 3e
* In this extension of exercise the operating slows for factor 1/10 if z-coordinate of grasping frame related to base frame is less than 0.3 m.
* Exercise 3e can be launched with following command: roslaunch urdf_tutorial urdf_tutorial_arm3.launch

