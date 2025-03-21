Project 2: Drone Simulation with Simplified Kalman Filter
==============

***EE4308 Autonomous Robot Systems***

**&copy; National University of Singapore**

# Table of Contents
[1&emsp;Administrative Matters](#1administrative-matters)

&emsp;[1.1&emsp;Submittables](#11submittables)

&emsp;[1.2&emsp;Download, Build and Run](#12download-build-and-run)

[2&emsp;Behavior Node](#2behavior-node)

&emsp;[2.1&emsp;Relevant Files for Behavior Node](#21relevant-files-for-behavior-node)

&emsp;[2.2&emsp;Implement `Behavior::cbTimer()`](#22implement-behaviorcbtimer)

&emsp;[2.3&emsp;Implement `Behavior::transition()`](#23implement-behaviortransition)

[3&emsp;Controller Node](#3controller-node)

&emsp;[3.1&emsp;Relevant Files for Controller Node](#31relevant-files-for-controller-node)

&emsp;[3.2&emsp;Implement `Controller::cbTimer()`](#32implement-controllercbtimer)

[4&emsp;Estimator Node](#4estimator-node)

&emsp;[4.1&emsp;Relevant Files for Estimator Node](#41relevant-files-for-estimator-node)

&emsp;[4.2&emsp;Prediction](#42prediction)

&emsp;&emsp;[4.2.1&emsp;Implement `Estimator::cbIMU()`](#421implement-estimatorcbimu)

&emsp;[4.3&emsp;Correction](#43correction)

&emsp;&emsp;[4.3.1&emsp;Implement `Estimator::cbSonar()`](#431implement-estimatorcbsonar)

&emsp;&emsp;[4.3.2&emsp;Implement `cbGPS()`](#432implement-cbgps)

&emsp;&emsp;[4.3.3&emsp;Implement `Estimator::getECEF()`](#433implement-estimatorgetecef)

&emsp;&emsp;[4.3.4&emsp;Implement `Estimator::cbMagnetic()`](#434implement-estimatorcbmagnetic)

&emsp;&emsp;[4.3.5&emsp;Implement `Estimator::cbBaro()`](#435implement-estimatorcbbaro)


# 1&emsp;Administrative Matters
Take note of the deadline and install all relevant software before proceeding to the lab. This project requires Lab 1 to be completed.

| Overview | Description |
| -- | -- |
| **Effort** | Team of 3. |
| **Signup for Teams** | On Canvas People, P2 Teams. |
| **Signup Presentation Slots** | On Canvas People, P2P. |
| **Signup Lab Slots** | On Canvas People, P2 Lab. |
| **Deadline** | W12 Mon, 23:59. | 
| **Submission** | Submit a zip file, recorded presentation and demonstration video, and a report to the P2 Assignment on Canvas. More details below. |
| **Software** | Refer to Lab 2. |
| **Lab Computers**| Refer to Lab 2. |

## 1.1&emsp;Submittables
For all filenames, label as `p2_team##` (**lowercase**), where `##` is the double-digit team number (e.g. for team 7, it is `p2_team07`). Submit the following files to the P2 assignment group.

| Component | Description |
| --- | --- |
| P2C | Zip the `ee4308/src` folder into `p2_team##.zip`. A well-programmed code is highly readable. Highly readable code consist of judicious comments, proper indentation, appropriately named variables, and minimal code. There is deliberate thought in considering the effects of every piece of code, such that possible bugs are prevented, and a balance between readability and optimization is achieved. |
| P2R | A report `p2_team##.pdf`. **Do not include your names in the report, only the matric numbers**. About 10 to 15 pages excluding front-matter and back-matter. In a good report, the existing algorithms are examined in detail and simple solutions are proposed to significantly improve the algorithms. Deliberate comparisons are made to compare solutions, and advice is given based on realistic situations. Experiments and methodologies to tune parameters are well designed and justified. The narrative is concise and clear. Any figures, tables and references are labelled. There is a title page and content page, and the report is tidy and well organized.  |
| P2P | A recorded presentation and demonstration `p2_team##.mp4`. 6-min of presentation with slides, and ~2+ min un-sped-up screen recording of the robot demonstration. 9-min of Q&A will be conducted at the end of the video playback during the presentation slot. In a good presentation, the narrative is based primarily on the solutions and analyses. A detailed understanding of the concepts and related nuances are demonstrated with concise explanations. Summaries and diagrams are used for comparisons and advice. A demonstration video of the required terminal and RViz layout is at https://youtu.be/WnRfxlB_j8Q.|

## 1.2&emsp;Download, Build and Run

Same as lab 2, except that a different launch file is used.
To run the project:
```bash
source ~/ee4308/install/setup.bash
ros2 launch ee4308_bringup proj2.launch.py
```

# 2&emsp;Behavior Node
A state machine that determines the behavior of the drone. 
For testing, `use_ground_truth` can be set to `true` in `proj2.yaml`. For the demonstration, `use_ground_truth` must be set to `false`.

The following requirements must be met:
- After taking off to an initial air position at cruise height, the drone must reach the following waypoints in order at *cruise height*: (i) move to the moving turtlebot, (ii) move to the turtlebot's current waypoint (its goal), (iii) back to the initial air position, and repeat from (i). After the turtlebot stops and the three-step cycle *completes*, the drone should then land. 
- The maximum threshold distance for reaching a waypoint is $0.3$ m.
- The cruise height is $5$ m. The drone can be expected to oscillate around this altitude.

## 2.1&emsp;Relevant Files for Behavior Node
1. `behavior.cpp`, found at `ee4308/src/ee4308_drone/src/`
2. `behavior.hpp`, found at `ee4308/src/ee4308_drone/include/ee4308_drone/`
3. `proj2.yaml`, found at `ee4308/src/ee4308_bringup/params/`

## 2.2&emsp;Implement `Behavior::cbTimer()`
This function calls the `transition()` function to find a new waypoint whenever the existing waypoint is reached. In addition, it continuously requests for a plan for the controller to follow. Using the constants representing the drone's state (see table), implement the state machine.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `TAKEOFF` | `int` | No | To read. Integer describing the state when the drone is taking off and moving to the initial air position. The initial air position is at cruise height above the initial ground position. | 
| `INITIAL` | `int` | No | To read. Integer describing the state when the drone is moving towards the initial air position. Has the same waypoint as `TAKEOFF`. | 
| `TURTLE_POSITION` | `int` | No | To read. Integer describing the state when the drone is moving towards the moving turtlebot. | 
| `TURTLE_WAYPOINT` | `int` | No | To read. Integer describing the state when the drone is moving towards the turtlebot's waypoint (a.k.a. goal). | 
| `LANDING` | `int` | No | To read. Integer describing the state when the drone is attempting to land at the initial ground position. | 
| `END` | `int` | No | To read. Integer describing the state when the drone prepares to stop moving after the landing completes. | 
| `odom_` | `nav_msgs::msg::Odometry` | No | To read the pose and twist information from `odom_.pose.pose` and `odom_.twist.twist` respectively. If the parameter `use_ground_truth` in `proj2.yaml` is set to `true`, the ground truth from the `odom` topic is used. If `use_ground_truth` is `false` (default, required for project), the estimated values from `estimator.cpp` is used. |
| `turtle_stop_` | `boolean` | No | To read and determine if the turtlebot has reached its final destination. If so, this variable will become `true`. `false` otherwise. |
| `reached_thres_` | `double` | Yes | To read the 3D threshold distance which determines if a waypoint has been reached. Must be set to no larger than `0.3` in `proj2.yaml`. |
| `state_` | `int` | No | To read integer constants describing the state. |
| `requestPlan()` | `function` | No | Use this function to request for a plan for the controller. |

The *limited* pseudocode is:

1. **Function** Behavior::cbTimer()
2. &emsp;**If** waypoint is reached **Then** 
3. &emsp;&emsp;**If** state is *taking-off* **Then**
4. &emsp;&emsp;&emsp;Transition to either *initial* state or *turtle-position* state.
5. &emsp;&emsp;**Else If** state is *initial* **Then**
6. &emsp;&emsp;&emsp;Transition to *landing* or *turtle-position* state depending on whether the turtlebot has stopped.
7. &emsp;&emsp;**Else If** state is *turtle-position* **Then**
8. &emsp;&emsp;&emsp;Transition to *turtle-waypoint*.
9. &emsp;&emsp;**Else If** state is *turtle-waypoint* **Then**
10. &emsp;&emsp;&emsp;Transition to *initial*.
11. &emsp;&emsp;**Else If** state is *landing* **Then**
12. &emsp;&emsp;&emsp;Transition to *end*.
13. &emsp;&emsp;**End If**
14. &emsp;**End If**
15. &emsp;Request a plan for the controller to follow.
17. **End Function**

## 2.3&emsp;Implement `Behavior::transition()`
The function updates the drone's waypoint and state based on the new state. You may merge this with the `Behavior::cbTimer()` function.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `new_state` | `int` | No | To read the new state (function argument). |
| `initial_x_` | `double` | No | To read the drone's initial $x$-coordinate on the ground. |
| `initial_y_` | `double` | No | To read the drone's initial $y$-coordinate on the ground. |
| `initial_z_` | `double` | No | To read the drone's initial $z$-coordinate on the ground. |
| `turtle_plan_` | `nav_msgs::msg::Path` | No | To read the current path of the turtlebot from `turtle_plan_.poses`. The front of the path contains the turtlebot's position. The end of the path contains the turtlebot's current waypoint. This plan will be unavailable if the turtlebot has not begun moving, or has completed all waypoints. |
| `cruise_height_` | `double` | Yes | To read the drone's cruise height. Must be set to `5.0` in `proj2.yaml`. |
| `state_` | `int` | No | To store integer constants describing the state. See the previous table for constants. |
| `waypoint_x_` | `double` | No | To store the desired $x$-coordinate of the drone's waypoint. |
| `waypoint_y_` | `double` | No | To store the desired $y$-coordinate of the drone's waypoint. |
| `waypoint_z_` | `double` | No | To store the desired $z$-coordinate of the drone's waypoint. |

Please implement this yourself.

# 3&emsp;Controller Node
Move the drone towards a lookahead point along a path.
In this version of pure pursuit, the drone is holonomic and can move in any direction. 
Therefore, the curvature from project 1 should not be calculated. 
You may notice some problems with the minimal implementation.

For testing, `use_ground_truth` can be set to `true` in `proj2.yaml`. For the demonstration, `use_ground_truth` must be set to `false`. 

`enable` can be set to `false` in `proj2.yaml` to disable the controller and gather sensor data for the estimator. The behavior node has to be manually disabled to prevent the drone from taking off.

The following requirements must be met:
- The drone must be rotating at $-0.3$ rad/s at all times.
- The maximum horizontal velocity (in the $x$ - $y$ plane) is $1$ m/s. 
- The maximum vertical velocity ($z$) is $0.5$ m/s.
- The maximum lookahead distance is $1$ m.

## 3.1&emsp;Relevant Files for Controller Node
1. `controller.cpp`, found at `ee4308/src/ee4308_drone/src/`
2. `controller.hpp`, found at `ee4308/src/ee4308_drone/include/ee4308_drone/`
3. `proj2.yaml`, found at `ee4308/src/ee4308_bringup/params/`

## 3.2&emsp;Implement `Controller::cbTimer()`

The following variables should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `odom_` | `nav_msgs::msg::Odometry` | No | To read the pose and twist information in `odom_.pose.pose` and `odom_.twist.twist` respectively. If the parameter `use_ground_truth` in `proj2.yaml` is set to `true`, the ground truth from the `odom` topic is used. If `use_ground_truth` is `false` (default, required for project), the estimated values from `estimator.cpp` is used. |
| `plan_` | `nav_msgs::msg::Path` | No | To read the points on the path from `plan_.poses`. The back of the path is where the drone's waypoint (goal) is. |
| `max_xy_vel_` | `double` | Yes | To read the maximum horizontal velocity in the $x$ and $y$ plane. Must be set to no larger than `1.0` in `proj2.yaml`. |
| `max_z_vel_` | `double` | Yes | To read the maximum vertical velocity along the $z$ axis. Must be set to no larger than `0.5` in `proj2.yaml`. |
| `yaw_vel_` | `double` | Yes | To read the required yaw velocity while the drone is moving. Must be set to `-0.3` in `proj2.yaml`. |
| `lookahead_distance_` | `double` | Yes | To read the lookahead distance. Must be set to no larger than `1.0` in `proj2.yaml`. |
| `publishCmdVel()` | `function` | No | Use this function to send command velocities. |

The *limited* pseudocode is:

1. **Function** Controller::cbTimer()
2. &emsp;**If** controller is disabled **Then**
3. &emsp;&emsp;**Return**
4. &emsp;**Else If** path is empty **Then**
5. &emsp;&emsp;Stop the drone in the air.
6. &emsp;&emsp;**Return**
7. &emsp;**EndIf**
8. &emsp;Find the closest point along the path.
9. &emsp;From the lookahead point by searching from the closest point.
10. &emsp;Determine the $x$ and $y$ velocities in the drone's frame to reach the lookahead point.
11. &emsp;Determine the $z$ velocity in the drone's frame to reach the lookahead point.
12. &emsp;Constrain the $x$ and $y$ velocities.
12. &emsp;Constrain the $z$ velocity.
15. &emsp;Move the drone in $x$, $y$, and $z$, and at the required yaw velocity.
17. **End Function**

# 4&emsp;Estimator Node
Implements the simplified Kalman Filter algorithm for estimating the drone's pose and twist using sensor data. 
Roll and pitch are ignored in the simplified motion model.
The estimator node publishes the estimated pose and twist into the `est_odom` topic, under the namespace `drone` (`/drone/est_odom`).

`verbose` can be set to `false` in `proj2.yaml` to disable the terminal output. The parameter must be set to `true` for the demonstration.

## 4.1&emsp;Relevant Files for Estimator Node
1. `estimator.cpp`, found at `ee4308/src/ee4308_drone/src/`
2. `estimator.hpp`, found at `ee4308/src/ee4308_drone/include/ee4308_drone/`
3. `proj2.yaml`, found at `ee4308/src/ee4308_bringup/params/`

## 4.2&emsp;Prediction
The prediction stage of the Kalman Filter uses Jacobians (first-order gradients) to approximate the process.
Two variables are being updated &mdash; $\mathbf{\hat{X}}$ describes the states being predicted, and $\mathbf{P}$ describes the covariance matrix (the uncertainty) of the states.

Let $\mathbf{F}$ and $\mathbf{W}$ be the Jacobian matrices (first-order partial derivatives) of the predicted state $\mathbf{\hat{X}}_ {k|k-1}$ (current prior) with respect to the previous state $\mathbf{\hat{X}}_{k-1|k-1}$ (previous posterior), and the current input $\mathbf{U}_k$.
The state update based on the first-order Taylor series approximation of the process $f$ is:

```math
\begin{align}
  \mathbf{\hat{X}}_{k|k-1}
      &= f(\mathbf{\hat{X}}_{k|k-1}, \mathbf{U}_k) \\
      &\approx \frac{\partial f}{\partial \mathbf{\hat{X}}_{k-1|k-1}} \mathbf{\hat{X}}_{k-1|k-1}
         + \frac{\partial f}{\partial \mathbf{U}_k} \mathbf{U}_k \\
      &= \mathbf{F}_k \mathbf{\hat{X}}_{k-1|k-1} + \mathbf{W}_k \mathbf{U}_k
\end{align}
```
Suppose $\mathbf{\hat{X}}$ estimates the position $p$ and velocity $v$ along or about an axis from some input $u_1$ and $u_2$, then by dropping the $\approx$ operator,
```math
\begin{align}
  \mathbf{\hat{X}}_{k|k-1}
      &= \mathbf{F}_k \mathbf{\hat{X}}_{k-1|k-1} + \mathbf{W}_k \mathbf{U}_k \\
    \begin{bmatrix} p_{k|k-1} \\ v_{k|k-1} \end{bmatrix}
      &= \begin{bmatrix}
        \frac{\partial p_{k|k-1}}{p_{k-1|k-1}} & \frac{\partial p_{k|k-1}}{v_{k-1|k-1}} \\
        \frac{\partial v_{k|k-1}}{p_{k-1|k-1}} & \frac{\partial v_{k|k-1}}{v_{k-1|k-1}}
      \end{bmatrix}
      \begin{bmatrix} p_{k-1|k-1} \\ v_{k-1|k-1} \end{bmatrix}
      + \begin{bmatrix}
        \frac{\partial p_{k|k-1}}{u_{1,k}} & \frac{\partial p_{k|k-1}}{u_{2,k}} \\
        \frac{\partial v_{k|k-1}}{u_{1,k}} & \frac{\partial v_{k|k-1}}{u_{2,k}}
      \end{bmatrix}
      \begin{bmatrix} u_{1,k} \\ u_{2,k} \end{bmatrix}
\end{align}
```

By modelling each state as a Gaussian variable, we find the covariance update equation as:
```math
\begin{align}
  \mathbf{P}_{k|k-1} &= \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^\top
    + \mathbf{W}_k \mathbf{Q}_k \mathbf{W}_k^\top
\end{align}
```
If $\mathbf{\hat{X}}$ contains $n$ states, then $\mathbf{P}$ is an $n\times n$ matrix. 

$\mathbf{Q}$ is a diagonal matrix describing the process noise from the input. 
In the example above, let $\sigma^2_1$ and $\sigma^2_2$ describe the noise (variance) of the inputs $u_1$ and $u_2$ respectively. Then,
```math
\begin{equation}
  \mathbf{Q}_{k} = \begin{bmatrix} \sigma^2_1 & 0 \\ 0 & \sigma^2_2 \end{bmatrix}
\end{equation}
```


## 4.2.1&emsp;Implement `Estimator::cbIMU()`
Extend the implementation in lab 2 to the other states in $x$, $y$ and $\psi$ (yaw).

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `msg` | `sensor_msgs::msg::Imu` | No | To read the linear acceleration force vector from `msg.linear_acceleration` and yaw velocity from `msg.angular_velocity`. `msg.orientation` should not be used. |
| `GRAVITY` | `double` | No | The acceleration due to gravity. |
| `dt` | `double` | No | $\Delta t$. To read the elapsed time from the last prediction. |
| `var_imu_x_` | `double` | Yes | $\sigma_{imu,x}^2$. To read the variance of IMU linear accelearation measurement along the drone's $x$-axis. |
| `var_imu_y_` | `double` | Yes | $\sigma_{imu,y}^2$. To read the variance of IMU linear accelearation measurement along the drone's $y$-axis. |
| `var_imu_z_` | `double` | Yes | $\sigma_{imu,z}^2$. To read the variance of IMU linear accelearation measurement along the drone's $z$-axis. |
| `var_imu_a_` | `double` | Yes | $\sigma_{imu,\psi}^2$. To read the variance of IMU angular velocity measurement about the drone's $z$-axis. |
| `Xx_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_x$. To store the drone's predicted $x$ position and velocity in the world frame. |
| `Xy_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_y$. To store the drone's predicted $y$ position and velocity in the world frame. |
| `Xz_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_z$. To store the drone's predicted $z$ position and velocity in the world frame. |
| `Xa_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_a$. To store the drone's predicted yaw and yaw velocity in the world frame. |
| `Px_` | `Eigen::Matrix2d` | No | $\mathbf{P}_x$. To store the predicted covariance matrix for the states in $\mathbf{\hat{X}}_x$. |
| `Py_` | `Eigen::Matrix2d` | No | $\mathbf{P}_y$. To store the predicted covariance matrix for the states in $\mathbf{\hat{X}}_y$. |
| `Pz_` | `Eigen::Matrix2d` | No | $\mathbf{P}_z$. To store the predicted covariance matrix for the states in $\mathbf{\hat{X}}_z$. |
| `Pa_` | `Eigen::Matrix2d` | No | $\mathbf{P}_ \psi$. To store the predicted covariance matrix for the states in $\mathbf{\hat{X}}_\psi$. |


Let's implement the prediction for the drone's states (position and velocity) along the world's $x$ axis.
Let $u_{x,k}$ and $u_{y,k}$ be the linear accelerations measured by the IMU in the drone's $x$ and $y$ axes. 
By *ignoring roll and pitch*, the accelerations $a_{x,k}$ and $a_{y,k}$ in the world frame are:
```math
\begin{align}
\begin{bmatrix} a_{x,k} \\ a_{y,k} \end{bmatrix} &=
  \begin{bmatrix} \cos{\psi} & -\sin{\psi} \\ \sin{\psi} & \cos{\psi} \end{bmatrix}
  \begin{bmatrix} u_{x,k} \\ u_{y,k} \end{bmatrix} \\
  &= \begin{bmatrix} \cos{\psi} & -\sin{\psi} \\ \sin{\psi} & \cos{\psi} \end{bmatrix} \mathbf{U}_{x,k}
\end{align}
```
The prediction for $\mathbf{\hat{X}}_x$ is
```math
\begin{align}
  \begin{bmatrix} x_{k|k-1} \\ \dot{x}_{k|k-1} \end{bmatrix}  &=
  \begin{bmatrix} x_{k-1|k-1} + \dot{x}_{k-1|k-1} \Delta t + \frac{1}{2}(\Delta t)^2 a_{x,k} \\
    \dot{x}_{k-1|k-1} + a_{x,k}\Delta t
   \end{bmatrix} \\
  \mathbf{\hat{X}}_{x,k|k-1} &= \mathbf{F}_{x,k} \mathbf{\hat{X}}_{x, k-1|k-1} + \mathbf{W}_{x,k} \mathbf{U}_{x,k}
\end{align}
```
Determine $\mathbf{F}_ {x,k}$ and $\mathbf{W}_ {x,k}$ from above. 


**Secondly**, we will implement the prediction on the covariance matrix for the $x$ axis. 
Let $\mathbf{Q}_x$ be the diagonal covariance matrix of the IMU noise in the IMU frame:
```math
\begin{equation}
  \mathbf{Q}_x = \begin{bmatrix} \sigma_{imu,x}^2 & 0 \\ 0 & \sigma_{imu,y}^2 \end{bmatrix}
\end{equation}
```
Using the matrices determined above, implement the covariance update:
```math
\begin{equation}
  \mathbf{P}_{x,k|k-1} = \mathbf{F}_{x,k} \mathbf{P}_{x,k-1|k-1} \mathbf{F}_{x,k}^\top + \mathbf{W}_{x,k} \mathbf{Q}_x \mathbf{W}_{x,k}^\top
\end{equation}
```


**Thirdly**, implement the prediction for the $y$ axis, which is similar to the $x$ axis.


**Fourthly**, we implement the prediction for the states along the world's $z$-axis, which is the same as Lab 2.
Let $a_{z,k}$ be the acceleration in the world frame, $u_{z,k}$ be the measured acceleration, and $G$ be the acceleration due to gravity. 
Determine if the following is an addition or subtraction of $G$:
```math
\begin{equation}
  a_{z,k} = u_{z,k} + ?
\end{equation}
```
Let $\mathbf{Q}_ z = \sigma^2_{imu,z}$, and $\mathbf{U}_ z$ be a $2\times 1$ vector to be determined.
Implement the process model for the states along the $z$ axis.


**Finally**, we implement the prediction for the drone's yaw and yaw velocities.
Let $u_{\psi,k}$ be the measured yaw velocity. Using $\Delta t$, $u_{\psi, k}$, determine the model below:
```math
\begin{equation}
  \mathbf{\hat{X}}_\psi =
  \begin{bmatrix} \psi_{k|k-1} \\ \dot{\psi}_{k|k-1} \end{bmatrix} =
  \begin{bmatrix} \psi_{k-1|k-1} + ? \\ ? \end{bmatrix}
\end{equation}
```
Subsequently, implement the process model for the yaw and yaw velocities.


## 4.3&emsp;Correction
The correction stage of the Kalman Filter is done asynchronously when a new sensor measurement becomes available.
The states and their uncertainties are updated (to current posterior) by minimising the least squared error between the sensor measurement and the prediction (current prior).
The equations are:
```math
\begin{align}
  \mathbf{K}_k &= \mathbf{P}_{k|k-1} \mathbf{H}_k^\top
    \left(
      \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^\top
      + \mathbf{V}_k \mathbf{R}_k \mathbf{V}_k^\top
    \right)^{-1} \\
  \mathbf{\hat{X}}_{k|k} &= \mathbf{\hat{X}}_{k|k-1} + \mathbf{K}_k
    \left(
      \mathbf{Y}_k - \mathbf{H}_k \mathbf{\hat{X}}_{k|k-1}
    \right) \\
  \mathbf{P}_{k|k} &= \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}
\end{align}
```

The Jacobian $\mathbf{H}$ is the partial derivative of the measurement with respect to the robot state (first-order approximation of the forward sensor model);
$\mathbf{V}$ is the Jacobian describing the transformation between the measurement frame and the frame where the sensor noise is measured;
and $\mathbf{Y}$ is the measurement, which contains the true state corrupted by some noise $\varepsilon$.

If there are no measurements between predictions, correction does not occur, and the current posterior state is simply the predicted state (current prior). There is nothing to do programmatically for this part.
```math
\begin{align}
  \mathbf{\hat{X}}_{k|k} &= \mathbf{\hat{X}}_{k|k-1} \\
  \mathbf{\hat{P}}_{k|k} &= \mathbf{\hat{P}}_{k|k-1} \\
\end{align}
```

In addition, for every prediction, multiple corrections from different sensors can occur. The subscript notations in the equations would cause confusion in this case, but as long as the states and their covariances are faithfully updated, the implementation would be correct.

### 4.3.1&emsp;Implement `Estimator::cbSonar()`
This should already be done in Lab 2. Implements the correction for the $z$ states when a sonar message is received.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `msg` | `sensor_msgs::msg::Range` | No | To read the sonar measurement from `msg.range`. |
| `var_sonar_` | `double` | Yes | $\sigma_{snr,z}^2$. To read the variance of sonar measurements along the world's $z$-axis. |
| `Ysonar_` | `Eigen::Vector3d` | No | $z_{snr}$. To store the sonar measurement. |
| `Xz_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_z$. To store the drone's corrected $z$ osition and velocity in the world frame. |
| `Pz_` | `Eigen::Matrix2d` | No | $\mathbf{P}_z$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_z$. |

As there is no transformation between the measurement and the sensor frame, $\mathbf{V}$ is one.
As the robot $z$ state shares the same frame as the sonar's measurement, 
the forward sensor model has nothing to transform,
and the Jacobian $\mathbf{H}$ of the measurement with respect to the robot state is $[1, 0]$.
To summarize,
```math
\begin{align}
  \mathbf{Y}_{snr,z,k} &= z_{snr} = z_{k|k-1}+ \varepsilon_{snr,k}\\

  \mathbf{H}_{snr,z,k} &= \frac{\partial \mathbf{Y}_{snr,z,k}}{\partial \mathbf{\hat{X}}_{z,k|k-1}}
    = \begin{bmatrix}
      \frac{\partial z_{snr}}{\partial z_{k|k-1}} & \frac{\partial z_{snr}}{\partial \dot{z}_{k|k-1}}
    \end{bmatrix}
    = \begin{bmatrix}1 & 0 \end{bmatrix}\\

  \mathbf{V}_{snr,z,k} &= 1 \\

  \mathbf{R}_{snr,z,k} &= \sigma^2_{snr,z}
\end{align}
```
Substitute the matrices into the correction update equations.

### 4.3.2&emsp;Implement `cbGPS()`
Implements the correction for the $x$, $y$, $z$ states when a GPS message is received.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `sin_lat` | `double` | No | $\sin(\varphi)$. To read the sine of the latitude. |
| `cos_lat` | `double` | No | $\cos(\varphi)$. To read the cosine of the latitude. |
| `sin_lon` | `double` | No | $\sin(\lambda)$. To read the sine of the longitude. |
| `cos_lon` | `double` | No | $\cos(\lambda)$. To read the cosine of the longitude. |
| `initial_ECEF_` | `Eigen::Vector3d` | No | $[x_{e,0}, y_{e,0}, z_{e,0}]^\top$. To read the initial ECEF coordinates. |
| `initial_position_` | `Eigen::Vector3d` | No | $[x_0, y_0, z_0]^\top$. To read the drone's initial coordinates in the world frame. |
| `var_gps_x_` | `double` | Yes | $\sigma_{gps,x}^2$. To read the variance of GPS measurements along the world's $x$-axis. |
| `var_gps_y_` | `double` | Yes | $\sigma_{gps,y}^2$. To read the variance of GPS measurements along the world's $y$-axis. |
| `var_gps_y_` | `double` | Yes | $\sigma_{gps,z}^2$. To read the variance of GPS measurements along the world's $z$-axis. |store the calculated GPS coordinates in the world frame (a.k.a. map frame). |
| `Ygps_` | `Eigen::Vector3d` | No | $[x_{gps}, y_{gps}, z_{gps} ]^\top$. To store the GPS measurements. |
| `Xx_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_x$. To store the drone's corrected $x$ position and velocity in the world frame. |
| `Xy_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_y$. To store the drone's corrected $y$ position and velocity in the world frame. |
| `Xz_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_z$. To store the drone's corrected $z$ position and velocity in the world frame. |
| `Px_` | `Eigen::Matrix2d` | No | $\mathbf{P}_x$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_x$. |
| `Py_` | `Eigen::Matrix2d` | No | $\mathbf{P}_y$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_y$. |
| `Pz_` | `Eigen::Matrix2d` | No | $\mathbf{P}_z$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_z$. |
| `getECEF()` | `function` | No | Use this function to calculate the ECEF coordinates. |

Let the calculated ECEF coordinates from the `getECEF()` function be $[x_e, y_e, z_e]^\top$, 
and the initial ECEF coordinates be $[x_{e,0}, y_{e,0}, z_{e,0}]^\top$.
The local NED coordinates $[x_n, y_n, z_n]^\top$ can be found as follows:
```math
\begin{align}
  \mathbf{R}_{e/n} &= \begin{bmatrix}
    -\sin(\varphi)\cos(\lambda) & -\sin(\lambda) & -\cos(\varphi)\cos(\lambda) \\
    -\sin(\varphi)\sin(\lambda) & \cos(\lambda) & -\cos(\varphi)\sin(\lambda) \\
    \cos(\varphi) & 0 & -\sin(\varphi) \\
  \end{bmatrix} \\
  \begin{bmatrix} x_n \\ y_n \\ z_n \end{bmatrix} &= \mathbf{R}_{e/n}^\top
    \left(
      \begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} -
      \begin{bmatrix} x_{e,0} \\ y_{e,0} \\ z_{e,0} \end{bmatrix}
    \right)
\end{align}
```

From the WGS84 convention, the $x$-axis points to the East, the $y$-axis points to the North, and the $z$-axis points up. 
Let the initial drone position in the world frame be $[x_0, y_0, z_0]^\top$.
To transform to the world frame,
```math
\begin{align}
  \mathbf{R}_{m/n} &= \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix} \\
  \begin{bmatrix} x_{gps} \\ y_{gps} \\ z_{gps} \end{bmatrix} &= \mathbf{R}_{m/n}
    \begin{bmatrix} x_n \\ y_n \\ z_n \end{bmatrix} +
    \begin{bmatrix} x_0 \\ y_0 \\ z_0 \end{bmatrix}
\end{align}
```

Consider the filter correction along the world's $x$-axis.
As there is no transformation between the measurement and the sensor frame, $\mathbf{V}$ is one.
As the robot state shares the same frame as the calculated GPS $x$-axis measurement, 
the forward sensor model has nothing to transform,
and the Jacobian $\mathbf{H}$ of the measurement with respect to the robot state is $[1, 0]$.
In summary,
```math
\begin{align}
  \mathbf{Y}_{gps,x,k} &= x_{gps} = x_{k|k-1} + \varepsilon_{gps,x,k}\\

  \mathbf{H}_{gps,x,k} &= \frac{\partial \mathbf{Y}_{gps,x,k}}{\partial \mathbf{\hat{X}}_{x,k|k-1}}
    = \begin{bmatrix}
      \frac{\partial x_{gps}}{\partial x_{k|k-1}} & \frac{\partial x_{gps}}{\partial \dot{x}_{k|k-1}}
    \end{bmatrix}
    = \begin{bmatrix}1 & 0 \end{bmatrix}\\

  \mathbf{V}_{gps,x,k} &= 1 \\

  \mathbf{R}_{gps,x,k} &= \sigma^2_{gps,x}
\end{align}
```
**Redo this for the $y$-axis and $z$-axis.**

### 4.3.3&emsp;Implement `Estimator::getECEF()`
Calculates and returns the ECEF coordinates for GPS.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `RAD_EQUATOR` | `double` | No | $a$. To read the equatorial radius. |
| `RAD_POLAR` | `double` | No | $b$. To read the polar radius. |
| `sin_lat` | `double` | No | $\sin(\varphi)$. To read the sine of the latitude. |
| `cos_lat` | `double` | No | $\cos(\varphi)$. To read the cosine of the latitude. |
| `sin_lon` | `double` | No | $\sin(\lambda)$. To read the sine of the longitude. |
| `cos_lon` | `double` | No | $\cos(\lambda)$. To read the cosine of the longitude. |
| `alt` | `double` | No | $h$. To read the altitude. |

The first step to using the GPS data is to calculate the ECEF coordinates $[x_e, y_e, z_e]^\top$. 
The coordinates are calculated by modelling Earth as an ellipsoid, 
where $N(\varphi)$ is the prime vertical radius of curvature, 
$a$ is the equatorial radius, $b$ is the polar radius, 
and $e^2$ is the square of the first numerical eccentricity. The equations are:
```math
\begin{align}
  e^2 &= 1 - \frac{b^2}{a^2} \\
  N(\varphi) &= \frac{a}{\sqrt{1 - e^2 \sin^2(\varphi)}} \\
  \begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} &=
    \begin{bmatrix}
      (N(\varphi) + h) \cos(\varphi) \cos(\lambda) \\
      (N(\varphi) + h) \cos(\varphi) \sin(\lambda) \\
      \left( \frac{b^2}{a^2} N(\varphi) + h \right) \sin(\varphi)
    \end{bmatrix}
\end{align}
```

### 4.3.4&emsp;Implement `Estimator::cbMagnetic()`
Implements the heading correction when a message from the magnetic compass arrives.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `msg` | `geometry_msgs::msg::Vector3Stamped` | No | To read the vector describing the magnetic force of Earth's magnetic field. In Gazebo, the vector points in the general $+x$ world direction even though north is pointing $+y$. However, this serendipitous coincidence means that there is no need to offset the calculated yaw since the drone always begins facing $+x$. |
| `var_magnet_` | `double` | Yes | $\sigma_{mgn,\psi}^2$. To read the variance of the yaw measurements. |
| `Ymagnet_` | `double` | No | $\psi_{mgn}$. To store the yaw measurement calculated from the force vector above. |
| `Xa_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_\psi$. To store the drone's corrected yaw and yaw velocity in the world frame. |
| `Pa_` | `Eigen::Matrix2d` | No | $\mathbf{P}_ \psi$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_\psi$. |

Let $f_{mgn}$ be an appropriate function that calculates the yaw from the force vector.
With the same justification as the previous sections,
```math
\begin{align}
  \mathbf{Y}_{mgn,\psi,k} &= \psi_{mgn} = f_{mgn}(x_{mgn}, y_{mgn}) + \varepsilon_{mgn,k} = \psi_{k|k-1} + \varepsilon_{mgn,k} \\

  \mathbf{H}_{mgn,\psi,k} &= \begin{bmatrix}1 & 0 \end{bmatrix}\\

  \mathbf{V}_{mgn,\psi,k} &= 1 \\

  \mathbf{R}_{mgn,\psi,k} &= \sigma^2_{mgn,\psi}
\end{align}
```

### 4.3.5&emsp;Implement `Estimator::cbBaro()`
This is optional for teams of 3, but required for teams of 4. Implements the $z$-axis correction when a message from the barometer arrives. 

As there can be a significant bias value in the barometer readings, a new state to estimate the barometer bias has to be augmented to $\mathbf{X}_z$, such that $\mathbf{\hat{X}}_z$ now contains three states, and $\mathbf{P}_z$ becomes a $3\times3$ matrix. 
In addition, all functions and matrcies that deal with $\mathbf{X}_z$ and $\mathbf{P}_z$ must be modified. 
This is where understanding the Jacobian $\mathbf{H}$ is important.

The following variables / functions should be used:
| Input Variable | Type | Parameter | Description |
| --- | --- | --- | --- |
| `msg` | `geometry_msgs::msg::PointStamped` | No | To read `msg.point.z` which contains the barometer altitude measurement. |
| `var_baro_` | `double` | Yes | $\sigma_{bar,z}^2$. To read the variance of the calculated yaw measurements. |
| `Ybaro_` | `double` | No | $z_{bar}$. To store the barometer altitude measurement. |
| `Xz_` | `Eigen::Vector2d` | No | $\mathbf{\hat{X}}_z$. To store the drone's corrected $z$ position and velocity in the world frame. |
| `Pz_` | `Eigen::Matrix2d` | No | $\mathbf{P}_ z$. To store the corrected covariance matrix for the states in $\mathbf{\hat{X}}_z$. |

In addition to resizing $\mathbf{P}_z$ to a $3\times 3$ matrix, $\mathbf{\hat{X}}_z$ should be adjusted to a $3\times 1$ vector to become:
```math
\begin{equation}
  \mathbf{\hat{X}}_z = \begin{bmatrix} z \\ \dot{z} \\ b_{bar} \end{bmatrix}
\end{equation}
```
where $b_{bar}$ is the augmented state representing the barometer bias.

The correction can then be summarized as:
```math
\begin{align}
  \mathbf{Y}_{bar,z,k} &= z_{bar} = z_{k|k-1} + b_{bar,k} + \varepsilon_{bar}\\

  \mathbf{H}_{bar,z,k} &= \begin{bmatrix} ? & ? & ? \end{bmatrix}\\

  \mathbf{V}_{bar,z,k} &= 1 \\

  \mathbf{R}_{bar,z,k} &= \sigma^2_{bar,z}
\end{align}
```
where $\mathbf{H}_{bar,z,k}$ is to be determined. 

