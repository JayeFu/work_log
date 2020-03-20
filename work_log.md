GOALS

- [x] seperate the robotic arm and spawn it in Gazebo
- [x] read the `src/iiwa_moveit/launch/move_group.launch` for controller addition
- [ ] add moveIt controller to control the movement of the robotic arm
- [x] replace the parallel mechanism with ur10 robot
- [ ] move upwards the position of the first joint of the arm or the mbx will collide with the ground

STEPS
1. sperate the robotic arm
   1. In order to reduce the work, I will not create a new package but just only add launch files to a existing package.
   2. The launch file will be added to `gazebo_example`
   3. The launch file will be called as `display_fwx.launch`

2. replace the parallel mechanism with ur10 robot
   1. macro `base_connectball_joint` contains 
      1. joint `${ns}base2connectball${chain_num}`
      2. link `${ns}connect_ball${chain_num}`
   2.  macro `connectball_chain_joint` contains
       1. joint `${ns}connectball2chain${chain_num}`
       2. link `${ns}chain${chain_num}_1`
   3. macro `chain_chain_joint` contains
      1. joint `${ns}chain2chain${chain_num}`
      2. link `${ns}chain${chain_num}_2`
   4. retain 
      1. upwards `connect`+`wx`
      2. downwards `car_link`
      3. `base_0` is a very small cubic
      4. `base` is `base_link.STL`
      5. no need for `common.gazebo.xacro`
   5. `JointTrajectoryController` can be used for controllers for multiple joints
      1. for tutorials:
         1. http://wiki.ros.org/joint_trajectory_action/Tutorials
         2. http://wiki.ros.org/Robots/TIAGo/Tutorials/trajectory_controller
   6. I have added `/mbx/` as prefix in `ur10_robot.urdf.xacro` on Feb 11th, 2020.
   7. `ur10_robot.urdf.xacro` successfully spawned in Rviz
   8. namespace has been modified properly.
   9. along z axis 0.055

ISSUES

1. When `neo_base_mp_400` is compiled, it will show 

```
CMake Error at /opt/ros/melodic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "neo_relayboard_v2"
  with any of the following names:

    neo_relayboard_v2Config.cmake
    neo_relayboard_v2-config.cmake

  Add the installation prefix of "neo_relayboard_v2" to CMAKE_PREFIX_PATH or
  set "neo_relayboard_v2_DIR" to a directory containing one of the above
  files.  If "neo_relayboard_v2" provides a separate development package or
  SDK, be sure it has been installed.
Call Stack (most recent call first):
  CMakeLists.txt:4 (find_package)
```
Resolution:

Delete `CATKIN_IGNORE` under `src/neo_driver/neo_relayboard_v2/`

2. What is `<node name="come_close_node" pkg="gazebo_example" type="come_close_node"/>` for?

Answer: If the distance between **mbx** ant **fwx** is less than 2.3, then publish `cmd_vel.linear.x=1` else `cmd_vel.linear.x=0`

3. What is `<node name="move_group_whole_system" pkg="iiwa_moveit" type="move_group_whole_system.py" output="screen"/>` is for?

Answer: Use **moveIt** like format to control the movement of the robotic arm. The only active function is `go_to_pose_goal`

```python
  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    model = GetModelStateRequest()
    model.model_name='mbx'
    model_state = get_model_state(model)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    link = GetLinkStateRequest()
    link.link_name='link_7'
    link_state = get_link_state(link)
    #eff_state = get_link_state1('wrist_3_link','world')
    
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.orientation.x = link_state.link_state.pose.orientation.x 
    #pose_goal.orientation.y = link_state.link_state.pose.orientation.y 
    #pose_goal.orientation.z = link_state.link_state.pose.orientation.z
    #pose_goal.orientation.w = link_state.link_state.pose.orientation.w
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0.707 
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 0.707
    pose_goal.position.x = model_state.pose.position.x - 1.8
    pose_goal.position.y = model_state.pose.position.y
    pose_goal.position.z = model_state.pose.position.z + 1
    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)
```

4. What to launch?
   1. `src/iiwa_moveit/launch/whole_system.launch`
   2. `src/iiwa_moveit/launch/move_group.launch`
   3. `src/iiwa_moveit/launch/close_planning.launch`

5. What is `src/iiwa_moveit/launch/whole_system.launch` for?
   1. Spawn robots
   2. `src/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.xacro` $\rightarrow$ `src/kuka_lbr_iiwa_support/urdf/iiwa_agv.urdf.xacro`
   3. `src/gazebo_example/urdf/mbx_model.urdf.xacro`
   4. include `src/neo_simulation/config/controller/launch/controller_utils.launch`
   5. load joint state configuration `iiwa_controller.yaml`
   6. spawn the above controller file

6. How to convert **xacro** to **urdf**?
```bash
$ roscore
$ rosrun xacro xacro -o model.urdf model.urdf.xacro
```
TIP: `--inorder` is default and thus not needed to include

Recommended to set 
```bash
alias xacro2urdf="rosrun xacro xacro -o model.urdf"
```
Then only to use `xacro2urdf model.urdf.xacro`

**TIP**: if any arguments for xacro then only to 

7. How to check `urdf` files?
   1. `check_urdf model.urdf`
   2. `urdf_to_graphiz model.urdf`

8. What is `src/neo_simulation/config/controller/launch/controller_utils.launch` for?
   1. publish `/calibrated` as `true`
   2. load joint state configuration file `src/neo_simulation/config/controller/joint_state_controller.yaml`
   3. spawn the above controller file

9. Why `because controller type 'position_controllers/JointTrajectoryController' does not exist.`?

Resolution: `sudo apt install ros-melodic-joint-trajectory-controller`

10. What is the difference between `robot_state_publisher` and `joint_state_publisher`?
    1. `joint_state_publisher` publishes `sensor_msgs/JointState` messages for a robot. The package reads the `robot_description` parameter, finds all of the non-fixed joints and publishes a `JointState` message with all those joints defined. 
    2.  `robot_state_publisher` uses the URDF specified by the parameter `robot_description` and the joint positions from the topic `joint_states` to calculate the forward kinematics of the robot and publish the results via `tf`. 

11. What to do before launch demo.launch
    1. `sudo apt install ros-melodic-trac-ik`
    2. `sudo apt install ros-melodic-moveit-*`

12. What is wrong with `panda_arm_hand.urdf`?
    1.  Please look at the `car.urdf`. I think maybe there is something wrong with the place of inertial tag and collision tag.
    2.  Look at [here](https://answers.ros.org/question/286022/why-my-model-urdf-doesnt-appear-in-gazebo/)

13. How to view `.dae` and `.stl` files?
    1.  `sudo apt install meshlab`
    2.  then `meshlab *.dae` or `meshlab *.stl`

14. ROS control in Gazebo
    1.  look at [here](http://gazebosim.org/tutorials/?tut=ros_comm)

15. **ALERT**: `*.dae` files cannot be used in Gazebo

16. Why `because controller type 'effort_controllers/JointPositionController' does not exist.`?
    1.  `sudo apt install ros-melodic-effort-controllers`
    2.  Then fantastic things happen.

17. Why I cannot control multiple joints with `effort_controllers/JointPositionController`
    1.  `effort_controllers/JointPositionController` is intended for single joint.
    2.  Look at [here](https://answers.ros.org/question/295500/create-a-yaml-config-file-to-use-jointpositioncontroller-in-order-to-control-a-robot-joints/)

18. `effort_controllers/JointPositionController` is available for control joints by sending `*/command` topic

19. `JointTrajectoryController` can be used for multiple joints
    1. see [here](http://wiki.ros.org/joint_trajectory_controller)

20. How to conveniently view a urdf model?
    1.  `roslaunch urdf_tutorial display.launch model:=*.urdf`

21. How to distinguish x y z axis in Rviz
    1.  red - x 
    2.  green - y 
    3.  blue -z

22. How to config the controllers?
    1.  in the yaml files, specify the joints and the corresponding controller type
    2.  ATTENTION: be careful about the namspace
    3.  if the controller is under a certain namespace, need to use format like

```xml
<node name="mbx_controller_spawner" pkg="controller_manager" type="spawner" 
  args="--namespace=/mbx 
    joint_state_controller 
    wx_controller 
    arm_controller"  
  respawn="false" output="screen"
/>
```
23. Does the `joint_state_controller` in config yaml files duplicate the `joint_state_publisher` in launch file?
    1.  ANSWER: **No!**
    2.  the `joint_state_controller` in yaml file is for simulation control and real robot control enough
    3.  But the `joint_state_publisher` in the launch file is a node for publish fake `joint_states` values
    4.  So in simulation, it is better to only retain `joint_state_controller`

24. A problem to be solved
    1.  I use `JointTrajectoryController` to control iiwa arm in Gazebo, but its movement is not synchronized in Rviz.
    2.  After checking the topic of `/mbx/arm_controller/state`, I find that the actual value is 0 while the desired value is right. Although in Gazebo, the arm is moved to the correct position.
    3.  Also, I tested the usage of `JointTrajectoryController` with rrbot in Gazebo. But its movement is synced in Rviz.
    4.  DO NOT KNOW WHY!
    5.  NOW the problem is caused by that I included two `gazebo_ros_control` plugin.
    6.  So TRY to make sure that there is only ONE `gazebo_ros_control` plugin in the xacro file.

25. How to launch multiple robot in one `launch` file?
    1.  [this website](https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/) shows how to spawn multiple robots in one launch file
    2.  NOTE: to successfully spawn the controllers please add `ns` in yaml loader as stated in the above website
    3.  In addition, remember to change the `robotParam` tag in `urdf.xacro` file! As show in [this website](https://blog.csdn.net/xu1129005165/article/details/53748636)
