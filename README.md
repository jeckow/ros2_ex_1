 ROS2 Exercise 1

## Overiew
This repository is for an exercise in ROS2 onboarding. The source folder contains the following packages

* **turtle_bringup** - contains the configuration and launch files required for the exercise
* **turtle_cpp_pkg** - C++ executables to be used in this exercise
* **turtle_interfaces** - custom messages and services folder
* **.vscode** - vscode folder for including the required file paths to run the program

\
**Naming convention**

* Launch files - turtle_ex1.launch*.xml
* CPP executables - turtlesim_ex*_action*.cpp

## References

Here are the reference files for each exercise:
\
\
**Exercise 1**
* turtlesim_ex1.cpp

**Exercise 1.1**
* turtle_ex1.launch*.xml
* turtlesim_ex1*.cpp

**Exercise 1.2**
* turtle2_config.yaml
* turtle_ex1_launch3.xml
* turlte_ex1_launch4.xml

**Exercise 2**
* turtlesim_ex2*

## General Description
**Exercise 1** - Creates a turtle (turtle1) in specific quadrant inside the turtlesim environment (where we also drawn the axes for conveniece). Another turtle (turtle2) will spawn in a different quadrant and everytime turtle1 is in its quadrant, turtle2 will rotate
\
\
**Exercise 2** - The difference with Exercise 1 is that of turtle2 rotating in place, it will instead trace a circular path as long as turtle1 is in turtle2's quadrant. A ROS2 service can also toggle the rotation on and off via terminal

## Notes
- update the file paths .vscode/c_cpp_properties.json to ensure that all packages will work as intended
- souce .bashrc in home or install/setup.bash in the main repository location upon building

## Graph view (rqt_graph)
rqt_graph provide an overview of how all of the nodes, topics, and groups are related. Here are the generalized graphs for each exercise:

* **Exercise 1**
<img width="1218" height="1182" alt="rosgraph" src="https://github.com/user-attachments/assets/0cb0f755-ed34-4bd5-bae1-acd25275ec44" />

* **Exercise 2**
  <img width="1530" height="1273" alt="rosgraph2" src="https://github.com/user-attachments/assets/59b4d1f2-33cb-4fce-b571-33075d9d2c3e" />

  

