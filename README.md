# ROS2_open_manipulator_control


## Introduction
The repository includes four implementations:
*    Forward Kinematics, Inverse kinamatics and pick and place 
     *   Forward kinematics subscriber node calculates end effector pose.
     *   Inverse kinematics subscriber node with a service client that receives the desired end effector pose from the user and returns joint positions as a response.
     *   Pick and place node executes a sequence where the robot moves to the calculated position, picks up the object, and place it; additionally, incorporate an intermediate gripper position above the object for effective picking and lifting maneuvers.

*    Looped Joint Position Update and cartesian control
     *   JK subscriber node with two services—one converts joint velocities to end effector velocities and the other converts end effector velocities to joint velocities.
     *   Incremental node supplys incremental position references to robot joints (q_ref = q_ref_old + delta_q * sampling_time), utilizing it as a velocity controller for joint position goals.
     *   Incremental node provides a constant positive velocity reference in specified direction in Cartesian space, convert it to joint space velocities using the Jacobian, feed it as a reference to the incremental position node, enabling the robot to move linearly in the in specified direction.

 *   PD Controller for End effector
     *   Implements a PD controller to regulate end effector position.
     *   Controller reads current joint position, receives position references, and publishes control efforts.


 *   moveit configuartion of open manipulator arm



Pick and place            |  moveit teleop-keyboard  |  Cartesian control
:-------------------------:|:-------------------------: |:-------------------------:
<img src="results/picknplace.gif" alt="Logo" width="200" height="300"> |  <img src="results/moveit.gif" alt="Logo" width="200" height="300"> | <img src="results/cartesian_control.gif" alt="Logo" width="400" height="300"> 


## Results

https://github.com/shambhurajmane/ROS2_open_manipulator_control/tree/main/results
