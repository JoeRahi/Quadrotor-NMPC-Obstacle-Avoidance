# Nonlinear Model Predictive Control (NMPC) for Quadrotor Trajectory Optimization

This repository contains a complete implementation of a Nonlinear Model Predictive Control (NMPC) algorithm for quadrotor trajectory optimization in cluttered 3D environments. The controller uses nonlinear quadrotor dynamics with small-angle approximations, RK4 integration, and a receding-horizon optimization structure implemented using CasADi + IPOPT.

The entire implementation is provided in a single MATLAB file.

Features

Full nonlinear quadrotor dynamics (12 states)

Small-angle approximation for simplified translational and rotational dynamics

Fourth-order Runge–Kutta (RK4) discretization

Receding-horizon NMPC with:

State-tracking cost

Control-effort minimization

Control-rate smoothing

Exponential obstacle-avoidance penalties

State and actuator constraints

3D trajectory visualization

Fully reproducible single-file implementation

File Structure
nmpc_quadrotor.m     # Main NMPC implementation (single file)
README.md            # Project description

Requirements
MATLAB

MATLAB R2020a or newer

CasADi

Download from: https://web.casadi.org/get/

Add CasADi to MATLAB path:

addpath(genpath('casadi-<version>-matlab'))

Quadrotor Model
States (12):
[x  y  z   φ  θ  ψ   vx vy vz   p  q  r]ᵀ

Inputs (4):
u = [u1  u2  u3  u4]ᵀ   # motor/rotor thrust inputs


The model includes thrust and torque generation, rotational dynamics, and linearized translational dynamics under small roll/pitch angles.

NMPC Formulation
Prediction horizon:
N = 10
dt = 0.15 s

Cost components:

State-tracking error

Control-effort cost

Control-rate smoothness

Exponential obstacle penalty

Constraints:

Altitude: 0.5 ≤ z ≤ 8 m

Roll and pitch: |φ| ≤ 0.25 rad, |θ| ≤ 0.25 rad

Rotor limits: 470 ≤ ui ≤ 530

How to Run

Install CasADi

Open MATLAB

Run:

run('nmpc_quadrotor.m')


The script automatically solves the NMPC problem, propagates the states with RK4, and plots the resulting trajectory and controls.

Results Summary

Stable quadrotor flight

Successful goal convergence

Smooth control inputs

Safe obstacle avoidance (soft penalties)

Full reproducibility using one script

License

MIT License
