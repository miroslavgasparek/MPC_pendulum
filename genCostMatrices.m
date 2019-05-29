function [H,G] = genCostMatrices(Gamma,Phi,Q,R,P,N)
% The function calculates the cost matrices H, G, so that the problem 
% of finding the optimal input can be written as the quadratic programming problem
% 
%     min     1/2 u_vec' * H * u_vec + (x0 - x_e)' * G' * u_vec
%   (u_vec)
% 
%   subject to  F * u_vec <= J * x0 + L * x_e + bb
% 
% where x_e is the target state
% 
% Gamma and Phi are the prediction matrices
% Q is the stage cost weight on the states, i.e. x' * Q * x
% R is the stage cost weight on the inputs, i.e. u' * R * u
% P is the terminal weight on the final state, i. e. x_N' * P * x_N

% Your code goes here
I_N_Rmat = eye(N);
I_N_Qmat = eye(N-1);
% Get R_mat
R_mat = kron(I_N_Rmat,R);


% Get Q_mat
Q_submat = kron(I_N_Qmat,Q);
Q_mat = [Q_submat, zeros(size(Q_submat,1), size(P,2));
         zeros(size(P,1), size(Q_submat,2) ), P];

% Get the H-matrix, weight on the input quadratic form     
H = (Gamma')*Q_mat*Gamma + R_mat;

G = (((Phi')*Q_mat*Gamma)');

end