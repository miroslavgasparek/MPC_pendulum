function [Hs,gs,Fs,bs,Js,Ls] = genSoftPadding(H,F,bb,J,L,S,rho,m)
% S = weight for quadratic cost of constraint violations
% rho = scalar weight for 1-norm of constraint violations
% m = number of inputs

% Get the number of inputs
N = size(H,1)/m;
I_N = eye(N);

% First define the Sbar
Sb = kron(I_N,S);

% Get the sizes of Sbar and H for further calculations
[row_Sb, col_Sb] = size(Sb);
[row_H, col_H] = size(H);

% Get the appropriate zero matrices
zeros_UR = zeros(row_H, col_Sb);
zeros_LL = zeros(row_Sb, col_H);

% Get the matrix Hs
Hs = [H, zeros_UR;
      zeros_LL, 2*Sb];
  
% Get the gs by multiplication
gs = rho*ones(row_Sb,1);

% Get the matrix I_S for the state constraints
[row_S, col_S] = size(S);
I_S = eye(row_S);

% Get the I_tilde matrix for the stage constraints
I_tilde = [-I_S; -I_S; zeros(m,col_S); zeros(m, col_S)];

% Get the I_bar matrix for the trajectory constraints
Ib = kron( eye(N), I_tilde);
[row_Ib, col_Ib] = size(Ib);

% Get the Fs matrix by combining F, Ib, appropriate identity matrix and 
% zero matrix
% Ibig = kron(ones(N), I_S);
Ibig = eye(col_Ib);
[row_Ibig, col_Ibig] = size(Ibig);
[row_F, col_F] = size(F);

Fs = [F, Ib; 
      zeros(row_Ibig, col_F), -Ibig];

% Get bs by complementing bb by zeros of appropriate length
bs = [bb; zeros(row_Ibig, size(bb,2))];
% Get Js by complementing J by zeros of appropriate length
Js = [J; zeros(row_Ibig, size(J,2))];
% Get Ls by complementing L by zeros of appropriate length
Ls = [L; zeros(row_Ibig, size(J,2))];


end