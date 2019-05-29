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

Where $g$ is acceleration due to the gravity, $l$ is the length of the rod, $b$ is the damping coefficient, and $u$ is the input, the normalized force.
