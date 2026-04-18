## 📈 Simulation Results

### 3D Trajectory
The optimized quadrotor trajectory in a cluttered 3D environment, showing smooth motion and safe obstacle avoidance.

<p align="center">
  <img src="nmpc_quadrotor.PNG" width="500"/>
</p>

---

### Position vs Time
Time evolution of the quadrotor position states, illustrating smoothed trajectory.

<p align="center">
  position_time.PNG
</p>

---

### Control Inputs (normalized rotor thrust commands)
The control inputs are dimensionless thrust commands normalized around the hover equilibrium; physical thrust values in Newtons are obtained internally through the actuator model and thrust coefficient.

<p align="center">
  Thrust_time.PNG
</p>

---

### Minimum Distance to Obstacles
Minimum signed distance between the quadrotor and nearby obstacles over time, demonstrating safe constraint satisfaction.

<p align="center">
  distance_obs.PNG
</p>
