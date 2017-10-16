
# Model Predictive Controller Project

## Intro

The model predictive controller requires a dynamical model of the process. In our case we have to include in our model how the steering, steering change, velocity and acceleration affects to the state of the vehicle. Having a dynamical model allows us to predict the future in a window of time. We can optimize the best route between the window of time. A PID controller has not this capacity.

## MPC Model

### The vehicle state

The stae is defined in the code by:

```cpp
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
```
x,y are position of the vehicle. Psi is the angle. V the velocity. CTE is the distance from the center and Epsi the error in psi.

### Actuators and cost function

IPOPT and CPPAD libraries are used to calculate an optimal trajectory and its associated actuation commands in order to minimize error with a third-degree polynomial fit to the given waypoints.
Actuator outputs are delta (steering angle) and acceleration. In MPC.cpp lines 79-124 are defined the model constraints by the dynamical model (like how to calculate future position from actual state).
In line MPC.cpp 246 is computed the solution taking in account the cost function and state boundaries. Boundaries are defined in line 245 section and costs in line 48-66.
I tried many differents combinations of cost function. The simplest one, with all weights=1 for all parameters was too inestable for driving. I added more weight to the parameters that I believed were more important for stability. The most important thing is to avoid big changes in acceleration and steering angle. A smoothest solution is less chaotic. A big change now requires a biggest change in the future and makes the vehicle to crash.
In MPC.cpp line 246 is solved the equation that minimizes the cost fitting a 3rd degree curve for the trajectory.

```cpp
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
```
defines the curve to be minimized. 

In:
```cpp
CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);
```
the equation is solved using the external numerical library IPOPT. This function calls fills the solution vector that is used to obtain the acceleration and delta parameters:


```cpp
  vector<double> result = {solution.x[delta_start], solution.x[a_start]};

```

### Timesteps
I chosed N=10 and dt = 0.1. That gives a time window of 1s with ten points to fit the trajectory. I tried other values like 20-0.05 and 5-0.2 but it seems 10-0.1 is good enough. Time windows cannot be too large because a simplified model like our losses precission with time.

### Latency
Be set N=10 and dt=0.1 so every step is 100ms long. To take in account the latency of the actuators (they don't transmit the order inmediatly) I included in MPC.cpp:

```cpp
AD<double> a0 = t>1?vars[a_start + t - 2]:vars[a_start + t - 1];
AD<double> delta0 = t>1?vars[delta_start + t - 2]:vars[delta_start + t - 1];
```
In the model constraints acceleration and delta has a delay of 100ms (they are applied to the model equations a time step later)


### Waypoints

The waypoints are transformed to the vehicle's perspective (see main.cpp lines 105-116). This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero. 
In main.cpp lines 147-160 are displayed the computed trajectory. In 162-173 the reference trajectory.

