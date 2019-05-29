function [Gamma, Phi] = genPrediction(A,B,N)
% Function generates the prediction matrices Gamma and Phi over the horizon
% of N steps. N is the horizon length.
% A and B are discrete-time state space matrices for x[k+1]=Ax[k]+Bu[k]
%
% hence for x_vec = [x_1; x_2; ... ; x_N] 
% and u_vec = [u_0; u_1; u_2; ... ; u_N-1 ], we have 
% 
% x_vec = Phi * x_0 + Gamma * u_vec
% 
% where Phi, Gamma are prediction matrices

% Pre-allocate empty matrices
Phi_mat = [];
Gamma = [];

% Add the values of the matrices to get Phi
for i = 1:N
    Phi_mat = [Phi_mat; A^i];
end

Gamma_init = [];

% Add the values of the matrices to get Gamma
for i = 1:N
    Gamma_init = [Gamma_init; (A^(i-1))*B];
end

Gamma_next = Gamma_init;
Gamma = Gamma_next;
for j = 1:N-1    
    Gamma_next = [zeros(size(B)); Gamma_next];
    Gamma_next = Gamma_next(1:end-(size(B,1)),:);
    Gamma = [Gamma, Gamma_next];
end


Phi = Phi_mat;
end