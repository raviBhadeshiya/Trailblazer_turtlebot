# Trailblazer: Reign of Gunters

The key task of this project was to implement object tracking algoritm and integrate it with ROS in order to find the object of interest at a given approximate location. The object found is then pushed to the ground using the arm mounted on turtlebot. In simulation the objects of interest and obstacles were not modeled. To improve the performance, we used a feature of ROS which is very useful that is *distributed architecture*. This feature allows the *master* node to be running on the robot whereas the node which is responsible for image processing runs on the workstation. This functionality helps to overcome the limited computational capability of the single board computer or netbook on the robot and performs all the heavy computations on the workstation. 

## Mandatory packages: arbotix, turtlebot_arm, and moveit

For arbotix: `sudo apt-get install ros-indigo-arbotix` 

For turtlebot_arm: Please follow the instructions [here](http://wiki.ros.org/turtlebot_arm/Tutorials/InstallationInstructions).

For moveit: `sudo apt-get install ros-indigo-moveit`

### Issue with OpenCV

Python API cannot find SIFT features module of OpenCV2.6.8(which comes with Indigo distro of ROS), thus needs an upgradation to OpenCV3.0.
Follow the instructions [here](https://github.com/amitrokh/adventure/wiki/Install-OpenCV) to install it. 

## Running the code (Simulation)

Please follow the following steps in order to run the code:
  1. Clone the repository
  2. Go to the root of your workspace and do `catkin_make`. 
  3. Launch a new terminal and type `roslaunch adventure_gazebo adventure_demo.launch`. This command will start the mandatory processes which are needed for simulation.
  4. Launch another terminal and typr `roslaunch adventure_recognition navigation.launch`. This command brings up the `move_base` server and initiates Advanced Monte-Carlo process. After this step you can assign the goal to the robot through RVIZ using *2D navigation goal* option on the tool bar. You can also check the movement of the arm mounted on the robot since this will also load OMPL library in RVIZ

Above mentioned steps are the checkpoints which ensure that the services are running properly and without any interruption. However, this project cannot be tested completely in the simulation environment since there are certain things which are not modeled in the simulation. Following is an example of the output obtained by following above mentioned steps.


![](https://github.com/nr-parikh/pioneer/blob/master/adventure_simulator/simulation.jpg)


## Running the code (Hardware)
Please ensure the following things:
  * Workstation and robot are able to communicate 
  * *Master* node always runs on the robot
  * On workstation ensure that ROS knows where the master node is running

Now, follow the steps mentioned below inorder to run the project successfully. Please make sure that *bashrc* file is being sourced every time a new terminal is launched.:

  1. Clone the repository in the worksapce on the workstation and do `catkin_make` at the root of the workspace
  2. Copy the **__adventure_bringup__** and **__adventure_gripper__** packages to the robot's workspace and don't forget to do `catkin_make` at the root of the workspace.
  3. SSH into the robot 
  4. Start the master node on the robot using `roscore`
  5. Launch the bringup file on the robot using `roslaunch adventure_bringup adventure.launch`
  6. On the workstation start terminal and launch navigation stack using `roslaunch advencture_recognition navigation.launch`
  7. Now, in another terminal run the file *object_recognition.py* using `rosrun adventure_recognition object_recognition.py`
  
The robot moves to the provided approximate coordinates, and looks for the object of interest(given as an input). If no object is found it rotates clockwise till the object comes under its field of view. If it finds the object in the field of view then it aligns itself in the proper orientation and moves to the proper distance at which the object is within the reach of arm. Then robot moves the arm to push the object to ground. This process is repeated for every provided coordinates. 
