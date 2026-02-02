[Nonlinear MPC Robotic Arm and EKF.pdf](https://github.com/user-attachments/files/24998084/Nonlinear.MPC.Robotic.Arm.and.EKF.pdf)# Nonlinear MPC - Robotic Arm Control (2-DOF)

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=simorxb/robotic-arm-2dof-mpc)

## Summary
This project demonstrates the use of Nonlinear Model Predictive Control (NMPC) to control a 2-DOF robotic arm. The robotic system is modeled using Simulink with Simscape Multibody, and the control strategy focuses on accurate tracking of the end-effector trajectory in the y-z plane. 

## Project Overview
This project explores advanced control techniques applied to a planar robotic manipulator with two rotational joints. The nonlinear dynamics of the robotic arm are modeled and controlled using NMPC, allowing for trajectory tracking without steady-state error.

### Key Features
- **Nonlinear MPC implementation** using MATLAB/Simulink.
- **Simscape Multibody simulation** of a 2-DOF planar robotic arm.
- **End-effector position control** in the y-z plane using predictive optimization.
- **Trajectory planning and disturbance rejection** tested in simulation.

## Robotic Arm Modeling

### System Diagram

The 2-DOF robotic arm consists of two rigid links with the following characteristics (see diagram on **page 2** of [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf)):

- Link 1 length ($a_1$): 0.2 m  
- Link 2 length ($a_2$): 0.15 m  
- Radius for both links: 0.01 m  
- Material: Steel (density = 7850 kg/m³)

### Dynamics
The robot dynamics are based on the following nonlinear model:

$$
M(\theta) \cdot \ddot{\theta} + C(\theta, \dot{\theta}) \cdot \dot{\theta} + G(\theta) + f(\dot{\theta}) = \tau
$$

Where:
- $\theta = [\theta_1; \theta_2]$
- $M, C, G$ represent the inertia matrix, Coriolis/centrifugal forces, and gravity terms, respectively.

These are derived from the multibody dynamics equations shown on **page 4** [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf).

## Nonlinear MPC Design

### Control Objectives
The NMPC controller is designed to:
- Track a time-varying end-effector position $[y, z]$
- Enforce torque saturation constraints
- Minimize control effort while maintaining performance

### Cost Function Output
The controller cost function is based on the deviation of the end-effector position from its reference in the $y,z$ plane:
- Output function: $[y, z]$ of end-effector
- Optimization over $\theta_1$, $\theta_2$ to reach desired $[y, z]$

### Controller Parameters
- **Sample time**: 0.1 s  
- **Prediction horizon**: 10  
- **Control horizon**: 4  
- **Torque constraints**: [-10 Nm, 10 Nm]  
- **Scaling factors**:
  - Manipulated variables: [2 2]  
  - Output variables: [1 1]  
- **Weights**:
  - Output: [1 1]  
  - Manipulated variables rate of change: [0.05 0.05]

See implementation details in code on **pages 6–10** [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf).

## Simulation Scenario

The system is initialized at:
- $\theta_1 = -\pi/2$
- $\theta_2 = 0$

The reference trajectory for the end-effector is updated at specified intervals:
1. $[0.2, 0.3]$ m at $t = 0$ s  
2. $[0.2, 0.6]$ m at $t = 1.5$ s  
3. $[-0.2, 0.6]$ m at $t = 3$ s

Trajectory plots and control results are provided on **pages 11 and 15** [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf).

## Simulink and Simscape Implementation
- Simulink model for the NMPC controller is included and illustrated on **page 12** [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf).
- Simscape Multibody is used for realistic physical simulation.
- Simulink result plots for joint angles, torques, velocities, and end-effector motion are shown on **pages 14–15** [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf).

## Results and Performance

### Highlights:
- No steady-state error observed despite no integral action in the control loop.
- Smooth control transitions between targets.
- Accurate trajectory tracking in a multibody environment.

See results in [Nonlinear MPC Robotic Arm.pdf](https://github.com/user-attachments/files/24698597/Nonlinear.MPC.Robotic.Arm.pdf):
- **Page 11**: MATLAB-based NMPC simulation plots  
- **Page 14**: Simulink simulation with Simscape Multibody  
- **Page 15**: Final end-effector trajectory plot

---

# Part 2 – Nonlinear MPC with Extended Kalman Filter (EKF)

## Summary
This second part extends the Nonlinear Model Predictive Control (NMPC) framework by integrating an Extended Kalman Filter (EKF) as a state estimator. The EKF enables state reconstruction and disturbance estimation when only limited measurements of the robotic arm are available, making the control architecture more realistic and robust.

Slides: [Nonlinear MPC Robotic Arm and EKF.pdf](https://github.com/user-attachments/files/24998091/Nonlinear.MPC.Robotic.Arm.and.EKF.pdf)



## Project Overview
In practical MPC applications, full state measurement is rarely available. This part of the project demonstrates how to combine NMPC with an Extended Kalman Filter to estimate unmeasured states and reject disturbances in a 2-DOF robotic arm.

The controller continues to regulate the end-effector position in the $y,z$ plane, while the EKF estimates joint positions, velocities, and unknown torque disturbances based solely on end-effector measurements.

## Extended Kalman Filter Design

### Estimation Objectives
The EKF is designed to:
- Estimate joint angles and angular velocities
- Estimate unknown torque disturbances acting on each joint
- Provide reliable state feedback to the NMPC controller

### Measurements Assumption
The estimator assumes that **only the end-effector position** $\begin{bmatrix} y \\ z \end{bmatrix}$ is measured. From these measurements, the EKF reconstructs the full state vector.

> **Note:**  
> This assumption is intentionally extreme and unrealistic. In a real system, at least joint positions are usually measured. The goal here is to stress-test the approach and highlight the power of model-based estimation.

### Augmented State Vector
The EKF and NMPC share an augmented state vector:

$$
x = \begin{bmatrix}
\theta_1 & \dot{\theta}_1 & \theta_2 & \dot{\theta}_2 & \tau_{1d} & \tau_{2d}
\end{bmatrix}^T
$$

Where:
- $\tau_{1d}, \tau_{2d}$ are estimated torque disturbances
- These states allow the controller to **reject disturbances**

## NMPC and EKF Integration
- The EKF provides estimated states to the NMPC at each sampling instant
- The NMPC uses these estimates to predict future system behavior
- Torque disturbance estimates improve robustness against modeling errors

The full Simulink architecture combining plant, NMPC, and EKF is shown in the **Simulink implementation diagram**.

## Simulation Setup

### Initial Conditions
- $\theta_1 = -\pi/2$
- $\theta_2 = 0$

### Reference Trajectory (End Effector)
1. $[0.2,\ 0.3]$ m at $t = 0$ s  
2. $[0.2,\ 0.6]$ m at $t = 1.5$ s  
3. $[-0.2,\ 0.6]$ m at $t = 3$ s  

The same trajectory used in Part 1 is retained to allow a direct comparison between:
- NMPC with full state availability
- NMPC with EKF-based state estimation

## Results and Performance

### Key Observations
- Accurate reconstruction of joint angles and velocities
- Effective estimation of torque disturbances
- No steady-state error despite the absence of integral action
- Smooth end-effector trajectory tracking using estimated states only

The results confirm that **there is no Model Predictive Control without a state estimator** when dealing with real-world systems.

## Key Takeaways
- EKF enables NMPC to operate with limited measurements
- Augmented disturbance states significantly improve robustness
- Predictive control and estimation work best when designed together
- The approach is directly transferable to real robotic systems

---



## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
