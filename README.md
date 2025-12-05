# Nonlinear Model Predictive Control (NMPC) for Quadrotor Trajectory Optimization

**A complete NMPC implementation for quadrotor trajectory optimization in cluttered 3D environments**

## 📋 Project Overview

This project implements a **Nonlinear Model Predictive Control (NMPC)** system for autonomous quadrotor navigation with obstacle avoidance. The controller uses **real-time optimization** to plan collision-free trajectories while respecting physical constraints and dynamics limitations.

**Key Problem Solved:** How to safely navigate a quadrotor (4 motor inputs controlling 6 degrees of freedom) through complex 3D obstacle fields to reach a target position.

## 🎯 Core Capabilities

### **Trajectory Optimization**
- Plans optimal paths through cluttered 3D environments
- Balances goal-reaching with obstacle avoidance
- Maintains physically feasible trajectories

### **Obstacle Avoidance**
- Uses **exponential barrier functions** for safe navigation
- Implements **Signed Distance Functions (SDF)** for cylinder obstacles
- Ensures minimum safe distances (0.25m) from all obstacles

### **Real-time Control**
- Solves NMPC optimization at ~0.15 seconds per step
- Uses **CasADi + IPOPT** for efficient nonlinear optimization
- Implements **warm-starting** for faster convergence

## 🏗️ Technical Architecture

### **System Dynamics**
The quadrotor is modeled with **12 states** using **small-angle approximations**:
- **Position**: x, y, z (3D coordinates)
- **Orientation**: φ (roll), θ (pitch), ψ (yaw)
- **Velocity**: v_x, v_y, v_z
- **Angular rates**: p, q, r

### **Control Formulation**
The NMPC solves:




System Dynamics
The quadrotor is modeled with 12 states using small-angle approximations:

Position: x, y, z (3D coordinates)

Orientation: φ (roll), θ (pitch), ψ (yaw)

Velocity: v_x, v_y, v_z

Angular rates: p, q, r

State vector: x = [x, y, z, φ, θ, ψ, v_x, v_y, v_z, p, q, r]ᵀ

Key simplifications for computational efficiency:

Euler angle kinematics: φ̇ = p, θ̇ = q, ψ̇ = r (valid for |φ|, |θ| < 0.25 rad)

Decoupled force equations: a_x = θ·T/m, a_y = -φ·T/m, a_z = T/m - g

This reduces computational complexity by ~40% while maintaining sufficient accuracy

Control Formulation
