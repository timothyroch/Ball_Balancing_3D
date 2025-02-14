# README: 3D Ball Balancing on an Inverted Hemispherical Bowl using Dual PID Control

This MATLAB simulation models a ball balancing on an inverted hemispherical bowl using dual PID control to maintain the ball's position near the bowl's apex.

## Table of Contents
1. [Overview](#overview)
2. [Mathematical Model](#mathematical-model)
3. [PID Control Mechanism](#pid-control-mechanism)
4. [Code Breakdown](#code-breakdown)
5. [Simulation Output](#simulation-output)

---

## 1. Overview
The code simulates a ball rolling on an inverted hemispherical bowl, where two PID controllers adjust the bowl's tilt to keep the ball centered. The ball moves based on gravitational forces, bowl tilt angles, and damping effects.

The simulation provides a real-time animation of:
- The ball's position and movement.
- The bowl's tilt angles as they vary over time.

---

## 2. Mathematical Model

The ball's motion in the bowl's local coordinate system is governed by these equations:

	x'' = (g/R)*x - g*phi_x - damping*x'
	y'' = (g/R)*y - g*phi_y - damping*y'

Where:
- `g` = gravitational acceleration (9.81 m/s²)
- `R` = bowl radius (0.2 m)
- `x, y` = ball's position on the bowl surface
- `x', y'` = ball's velocities
- `x'', y''` = ball's accelerations
- `phi_x, phi_y` = bowl tilt angles around the y- and x-axes respectively

The bowl surface is defined as:

	z = sqrt(R² - x² - y²) - R

At each time step, the bowl is rotated with rotation matrices around the x- and y-axes:

	R_world = R_y(phi_x) * R_x(phi_y)

This ensures the ball responds naturally to the bowl's tilt.

---

## 3. PID Control Mechanism

The PID controllers compute the tilt angles `phi_x` and `phi_y` to bring the ball back to the top. For both x and y directions, the controller calculates:

	phi = Kp * error + Ki * integral_error + Kd * derivative_error

The gains used in this simulation are:
- `Kp = 15` (Proportional Gain)
- `Ki = 1` (Integral Gain)
- `Kd = 5` (Derivative Gain)

### PID Behavior
- **Proportional**: Tilts the bowl based on the ball's distance from the center.
- **Integral**: Adjusts based on accumulated error (useful for eliminating steady-state drift).
- **Derivative**: Reacts to changes in error, preventing overshoot.

---

## 4. Code Breakdown

### 4.1 Initialization
The script starts by clearing the workspace and setting up parameters:
- Time step, total duration.
- Physical constants like `g`, `R`, and `damping`.
- PID parameters.
- Initial ball position and velocities.

### 4.2 Bowl and Ball Surface Precomputation
- The bowl is modeled using spherical coordinates.
- The ball is represented by a small sphere that moves along the bowl surface.

### 4.3 Visualization Setup
The figure contains two subplots:
1. **3D Animation**: Shows the bowl and ball in real-time.
2. **Tilt Angles Plot**: Displays the evolution of `phi_x` and `phi_y` over time.

### 4.4 Main Simulation Loop
For each time step:
1. **PID Control**: Calculate tilt angles from ball position.
2. **Ball Dynamics Update**: Update the ball's position and velocity using the motion equations.
3. **Bowl Rotation**: Compute the world coordinates from the local coordinates.
4. **Ball Position Update**: Recompute ball position based on updated bowl surface and tilt.
5. **Graphics Update**: Refresh the surface plots for bowl and ball.

### 4.5 Output
The loop runs until the final time is reached, simulating the dynamic stabilization process.

---

## 5. Simulation Output

The simulation produces:
- A 3D animation of the bowl tilting as the ball moves.
- The ball's motion gradually settles near the top of the bowl.
- A plot of the tilt angles `phi_x` and `phi_y` over time.

### Expected Behavior:
- The ball starts near the edge and moves toward the center.
- The bowl tilts to counteract the ball's movement.
- The tilt angles fluctuate initially but stabilize as the ball approaches the center.

---

## How to Run the Code
Ensure MATLAB is installed. Copy the code into a MATLAB script file (e.g., `BallBalancingPID.m`) and run it.

**Command:**
```matlab
run('BallBalancingPID.m')
```

---

This concludes the explanation of the ball-balancing simulation using dual PID control in MATLAB. Adjust the PID gains or initial conditions to experiment with the system's response characteristics.

