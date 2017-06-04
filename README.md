# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
Robbie Edwards May 2017 submission

## Basic Build Instructions

1. Compile: `cmake .. && make`
2. Run it: `./mpc`.

## Video

Here is a link to the project video at 57mph.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ZCe8aLxr1lk" target="_blank"><img src="http://img.youtube.com/vi/ZCe8aLxr1lk/0.jpg"
alt="MPC at 57mph" width="240" height="180" border="10" /></a>

## Solution

The framework and part of the code was adapted from the Udacity MPC quiz. This provided the setup of the IPOPT optimizer including the cost function and constraints.

The following state variables were used with MPC, per the mpc lessons:

- x: x coordinate (car frame)
- y: y coordinate (car frame)
- psi: heading angle (car frame)
- v: speed
- cte: cross track error
- error_psi: heading error

The controls:

- delta: steering angle
- a: acceleration / throttle value

The vehicle kinematics from the mpc course material were used as contrainst between timesteps of each variable.

```c++
x_t1 = (x_t + v_t * cos(psi_t) * dt);
y_t1 = (y_t + v_t * sin(psi_t) * dt);
psi_t1 = (psi_t + v_t * delta_t / Lf * dt);
v-t1 = (v_t + a_t * dt);
cte_t1 = ((cte_t - y_t) + (v_t *sin(epsi_t) * dt));
epsi_t1 - ((psi_t - psides_t) + v_t * delta_t / Lf * dt);
```

The controls were constrained to the acceptable input values for the simulator of -25..25 deg for steering and -1..1 throttle setting.


### Optimizer cost function

A similar cost function to the course material was used. Weights on each cost term were used and tuned to provide accpetable driving behavior. In general, the weights were not adjusted significantly between the velocity set points which were attempted.

The cost function considered:

1. The cross track error over each predicted state / timesteps
2. The heading error over each timesteps
3. error from the desired velocity setpoint
4. Use of the steering actuators
5. Use of the throttle actuators
6. Difference between timesteps of the steering actuator (smoothness)
7. Difference between timesteps of the throttle actuator (smoothness)

### Coordinates

Telemtry was provided in the global coordinate frame, this was translated and rotated into the vehicle reference frame in main.cpp line 111. The optimizer and polynomial fitter were run in the vehicle reference frame coordinates.

### Timesteps and delta_t

It was found that for a given set of cost function weights, the timestep dt could be adjusted somewhat proportionally to the vehicle velocity to acheive a roughly constant prediction distance. That is, decrease dt with increasing velocity.

For a fixed speed, by decreasing dt too much, the car would become over-responsive and jittery. It would often drive off the track.

By using a dt too large, the car would not be responsive enough to drive around the tighter corners.

For 60mph and N = 15:

 - dt = 0.005s: car too jittery, drives off course
 - dt = 0.03s: good performance
 - dt = 0.01s: car's initial control too slow, unresponsive, drives off course. Large variations in control output between timesteps

For the number of timesteps:
Using too few timesteps would result in a short prediction distance, with inadequate fitting. The car would often quickly drive off the track.

Using too many time steps would result in the MPC predicting farther than the vehicle's telemetry as well as evaluating the fitted polynomial outside the region in which it was fit.  The car would drive off the course.

For 60mph and dt = 0.03s

 - N = 5: too few, car immediately drives off course
 - N = 15-20: good performance
 - N = 30: some overfitting / funny trajectories, car still drives course
 - N = 50: overfitting, beyond valid polynomial, car leaves course.

 The total prediction distance of N*dt*v is important to consider. It should not be longer than the telemetry.

## Latency

Latency was handled at main.cpp line 135 by using the vehicle kinematics in the vehicle frame of reference to adjust the mpc solver input state forward by the latency time of 100ms. This means the solver should be generating control output for when the vehicle is ready to respond to it.

## Improvements

This MPC implementation could benefit from much more time tuning the cost function weights as well as other parameters. Perhaps from automatic tuning of the cost function weights.  It would also be interesting to apply other terms to the cost function in future work. The vehicle dynamics could also be incorporated in place of the kinematics, but the dynamics used in the simulator would have to be known.
