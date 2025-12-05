# NMPC Quadrotor Trajectory Optimization

This repository presents a **Nonlinear Model Predictive Control (NMPC)** framework for quadrotor trajectory optimization in cluttered 3D environments. The quadrotor is modeled using nonlinear dynamics with small-angle approximations and discretized with a fourth-order Runge–Kutta integrator. The NMPC formulation incorporates state-tracking, control-effort minimization, control-rate smoothing, and exponential barrier functions for obstacle avoidance, while explicitly enforcing state and actuator constraints.

---

## 📋 Abstract

The proposed NMPC controller successfully guides a quadrotor to its goal while safely navigating around obstacles. Simulation results, plots, and parameters are provided to facilitate reproducibility.

**Keywords:** Quadrotor Control, Nonlinear Model Predictive Control, Trajectory Optimization, Obstacle Avoidance, Motion Planning, Runge-Kutta Methods

---

## 🎯 Core Capabilities

- **Trajectory Optimization:** Plans optimal paths through cluttered 3D environments while maintaining physically feasible trajectories.  
- **Obstacle Avoidance:** Exponential barrier functions using Signed Distance Functions (SDF) for cylindrical obstacles.  
- **Real-time Control:** NMPC solved at ~0.15 s per step using CasADi + IPOPT, with warm-starting for faster convergence.  
- **Visualization:** 3D animated quadrotor showing real-time position, orientation, and obstacles.  

---

## 🏗️ Technical Architecture

### Quadrotor Dynamics
- **States (12):**  
  - Position: x, y, z  
  - Orientation: φ (roll), θ (pitch), ψ (yaw)  
  - Velocity: v_x, v_y, v_z  
  - Angular rates: p, q, r  
- Small-angle approximation for attitude dynamics  
- Force coupling: a_x = θ·T/m, a_y = -φ·T/m  
- Euler-angle kinematics: φ̇ = p, θ̇ = q, ψ̇ = r  

### NMPC Formulation
- Horizon: **N = 10 steps (~1.5 s)**  
- Cost function: state-tracking, control effort, control-rate smoothing  
- Obstacle avoidance with exponential barrier functions  
- Actuator & state constraints:
  - Rotor thrust limits  
  - Roll/Pitch angle limits  
  - Z-position limits  
- Warm-start using previous control sequence  
- Solver: **CasADi + IPOPT**

---

## ⚙️ Installation

1. Install **MATLAB R2020a or newer**.  
2. Download and install **CasADi**: [https://web.casadi.org/get/](https://web.casadi.org/get/)  
3. Add CasADi to MATLAB path:  
addpath(genpath('casadi-<version>-matlab'))
4. git clone https://github.com/JoeRahi/NMPC-Quadrotor.git
5. Open MATLAB and set the cloned folder as the working directory.


How to Run

Open main.m (or the main NMPC script).

Run the script in MATLAB. The simulation will:

Compute the NMPC trajectory

Animate the quadrotor in 3D with obstacles

Generate plots for: position, velocity, Euler angles, control inputs, control effort, computation time, obstacle distances, and position error
