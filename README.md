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

The implemented dynamical system has the following forom: 
\begin{align}
\frac{d}{dt}x_{1} = x_{2} \\
\frac{d}{dt}x_{2} = -frac{g}{l} sin(x_{1}) - b x_{2} + u
\end{align}
