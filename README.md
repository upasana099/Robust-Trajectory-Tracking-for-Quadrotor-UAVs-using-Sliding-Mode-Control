# Robust Trajectory Tracking for Quadrotor UAVs using Sliding Mode Control

The goal of this project is to develop a robust control scheme that enables a
quad-rotor to track desired trajectories despite the presence of external disturbances. To achieve this, we will use the Crazyflie 2.0 Micro Aerial Vehicle as
a testing platform in a simulated environment using the Gazebo simulator and
ROS middleware


# Waypoint Tracking

In this part, the quadrotor needs to visit a sequence of waypoints in a specific order while maintaining zero velocity and acceleration at each waypoint. The waypoints are as follows:

* p0 = (0, 0, 0) to p1 = (0, 0, 1) in 5 seconds
* p1 = (0, 0, 1) to p2 = (1, 0, 1) in 15 seconds
* p2 = (1, 0, 1) to p3 = (1, 1, 1) in 15 seconds
* p3 = (1, 1, 1) to p4 = (0, 1, 1) in 15 seconds
* p4 = (0, 1, 1) to p5 = (0, 0, 1) in 15 seconds

![01](https://github.com/upasana099/Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control/assets/89516193/4866158e-6279-4c2a-8c4e-94393053b0a8)


# Sliding Mode Control Laws

Designed boundary layer-based sliding mode control laws to enable the quadrotor to track desired trajectories for z, ϕ, θ, and ψ coordinates in altitude and attitude control. Considered zero angular velocities/accelerations and the desired yaw angle ψ during the motion.

#  ROS Node Implementation

Implemented a ROS node in Python to evaluate the performance of the sliding mode control design on the Crazyflie 2.0 quadrotor in Gazebo. The script incorporated the trajectories generated in Part 1 and integrated the sliding mode control laws formulated in Part 2.

#  Trajectory Logging

Saved the actual trajectory as a log.pkl file in the scripts directory when the program was shut down. Used the visualize.py script to visualize the trajectory by loading the saved log file         

# Results

![02](https://github.com/upasana099/Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control/assets/89516193/395510c3-d4d7-486f-bf31-0c90524e6d88)


Actual Position Trajectory in 3D


https://github.com/upasana099/Robust-Trajectory-Tracking-for-Quadrotor-UAVs-using-Sliding-Mode-Control/assets/89516193/cf0be976-f1a2-4c55-80a4-486585992f6a


Crazyflie 2.0 Simulation Video 



