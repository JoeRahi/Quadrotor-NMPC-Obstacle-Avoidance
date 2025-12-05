# NMPC Quadrotor Trajectory Optimization

This repository presents a **Nonlinear Model Predictive Control (NMPC)** framework for quadrotor trajectory optimization in cluttered 3D environments. The quadrotor is modeled using nonlinear dynamics with small-angle approximations and discretized with a **fourth-order Runge–Kutta integrator**. The NMPC formulation incorporates **state-tracking, control-effort minimization, control-rate smoothing, and exponential barrier functions for obstacle avoidance**, while explicitly enforcing state and actuator constraints.

---

## 📋 Abstract

The proposed NMPC controller successfully guides a quadrotor to its goal while safely navigating around obstacles. Simulation results and parameters are provided to facilitate reproducibility.

**Keywords:** Quadrotor Control, Nonlinear Model Predictive Control, Trajectory Optimization, Obstacle Avoidance, Motion Planning, Runge-Kutta Methods

---

## 🎯 Core Capabilities

- **Trajectory Optimization:** Plans optimal paths through cluttered 3D environments while maintaining feasible trajectories.
- **Obstacle Avoidance:** Exponential barrier functions with Signed Distance Functions (SDF) for cylindrical obstacles.
- **Real-time Control:** NMPC solved at ~0.15 s per step using **CasADi + IPOPT**, with warm-starting for faster convergence.
- **Visualization:** 3D animated quadrotor with real-time position, orientation, and obstacle rendering.

---

## 🏗️ Technical Architecture

### Quadrotor Dynamics

- 12 states: position `(x, y, z)`, orientation `(φ, θ, ψ)`, velocity `(v_x, v_y, v_z)`, angular rates `(p, q, r)`
- Small-angle approximation for attitude dynamics
- Force coupling: `a_x = θ·T/m`, `a_y = -φ·T/m`
- Euler-angle kinematics: φ̇=p, θ̇=q, ψ̇=r

### NMPC Formulation

- Horizon: `N = 10` steps (1.5 s)
- State-tracking, control effort, and control-rate penalties
- Obstacle avoidance with exponential barrier functions
- Actuator and state constraints:
  - Rotor thrust limits
  - Roll/Pitch angle limits
  - Z-position limits
- Warm-start using previous control sequence
- Solver: **CasADi + IPOPT**

---

## ⚙️ Installation

1. Install MATLAB R2020a or newer.
2. Download and install CasADi: [https://web.casadi.org/get/](https://web.casadi.org/get/).
3. Add CasADi to MATLAB path:

```matlab
addpath(genpath('casadi-<version>-matlab'))
