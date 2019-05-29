function [DD,EE,bb]=genTrajectoryConstraints(Dt,Et,bt,N)
% The function generates the trajectory constraints for the linear (or
% linearized) system, such that the constraints in the form of 
% 
% Dt * x(k) + Et * u(k) <= bt
% 
% can be written as 
% 
% DD * x_vec + EE * u_vec <= bb
% 
% where x_vec, u_vec are the vectors of states and inputs over the
% prediction horizon respectively

% Generate the identity matrix of the appropriate size
I_N = eye(N);
% Generate the vector of ones of the appropriate size
one_vec = ones(N,1);


% Generate the matrix DD
DD = kron(I_N, Dt);

% Generate the matrix EE
EE = kron(I_N, Et);

% Generate the vector bb
bb = kron(one_vec,bt);

end