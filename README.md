# Nonlinear MPC - Robotic Arm Control (2-DOF)

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

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
