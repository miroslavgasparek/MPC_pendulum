# MPC_pendulum
This repository contains the MATLAB scripts for the implementation of the Model Predictive Control for the trajectory tracking
in the system of a nonlinear pendulum. 

This approach combines the linearization of the nonlinear dynamical system, which is subsequently combined with the application
of the condensed approach to the construction and solution of the Quadratic Programming problem over the length of the
prediction horizon, as described in
Jerez, J. L., Kerrigan, E. C., & Constantinides, G. A. (2011). A Condensed and Sparse QP Formulation for Predictive Control. https://doi.org/10.0/Linux-x86_64

Of course, the pendulum implementation can be easily amended so that different systems can be implemented. Code is well commented
and you should be able to read through that, given that you have the understanding of the basic Quadratic Programming,
linear algebra, and dynamical systems.

The implemented dynamical system has the following form:


![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Balign*%7D%20%5Cfrac%7Bd%7D%7Bdt%7Dx_%7B1%7D%20%26%3D%20x_%7B2%7D%20%5C%5C%20%5Cfrac%7Bd%7D%7Bdt%7Dx_%7B2%7D%20%26%3D%20-%20%5Cfrac%7Bg%7D%7Bl%7D%20sin%28x_%7B1%7D%29%20-%20b%20x_%7B2%7D%20&plus;%20u%20%5Cend%7Balign*%7D)


Where x1 is the angle of the pendulum form the vertical, x2 is the angular velocity, g = 9.81 m/s^2 is the acceleration due to gravity, l=0.1 m is the length of the rod, b=0.2 rad/s is the damping coefficient, and u is the input, the normalized force. Our aim is to get to the constant angle of 0.5 rad (x1) and 0.0 (x2). We impose the constraints on angular velocity, so that minimum angular velocity is -4 rad/s and maximum angular velocity is 4 rad/s, while the minimum input (normalized force, a.k.a. acceleration) is -20 rad/s^2 and maximum input is 80 rad/s^2.

In this case, we run the simulation for 0.5 s with the prediction horizon of 0.04 s, while the sampling period is 0.001 s. We place quite high weight on the position, medium weight on the angular velocity, and very low weight on the input. Details can be found in the file `runPendulumMPC.m`.

The results of the simulation are displayed below: 
![alt text]https://github.com/miroslavgasparek/MPC_pendulum/blob/master/pendulum_traj_tracking_MPC.jpg
