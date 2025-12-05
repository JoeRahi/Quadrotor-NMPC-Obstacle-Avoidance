# Nonlinear Model Predictive Control (NMPC) for Quadrotor Trajectory Optimization

This repository contains a full implementation of a **Nonlinear Model Predictive Control (NMPC)** framework for quadrotor trajectory tracking and obstacle avoidance in cluttered 3D environments.  
The controller is implemented in **one MATLAB file**, using **CasADi + IPOPT** for real-time optimization and **RK4 integration** for accurate dynamics.

---

## 🚀 Features

- **Complete 12-state quadrotor model**
- **Small-angle nonlinear dynamics** for improved numerical stability
- **Fourth-order Runge–Kutta (RK4)** discretization
- **Receding-horizon NMPC**
  - state tracking  
  - control-effort penalty  
  - control-rate smoothing  
  - exponential barrier functions for obstacle avoidance
- **State & actuator constraints**
- **3D trajectory visualization**
- **Single MATLAB file (easy to read & reproduce)**

---

## 📁 Repository Structure

