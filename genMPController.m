%% Miroslav Gasparek

function [u,status,iA1] = genMPController(H,G,F,bb,J,L,x,xTarget,m,iA)
% H       - quadratic term in the cost function (Linv if using mpcqpsolver).
% G       - matrix in the linear term in the cost function.
% F       - LHS of the inequalities term.
% bb      - RHS of the inequalities term.
% J       - RHS of inequalities term.
% L       - RHS of inequalities term. Use this to shift constraints around target point
% x       - current state
% xTarget - target equilibrium point.
% m       - Number of inputs.
% iA      - active inequalities, see doc mpcqpsolver
%
% u is the first input vector u_0 in the sequence [u_0; u_1; ...; u_{N-1}]; 
% In other words, u is the value of the receding horizon control law
% evaluated with the current state x0 and target xTarget

% Please read the documentation on mpcqpsolver to understand how it is
% supposed to be used. Use iA and iA1 to pass the active inequality vector 

opt.MaxIter = 200;
opt.IntegrityChecks = false;%% for code generation
opt.FeasibilityTol = 1e-3;
opt.DataType = 'double';
%% your code starts here
% Compute the Linv, an inverse of the lower-triangular Cholesky
% decomposition of Hessian matrix H which is given

%% Only compute if needed!!!
% [Lmat,p] = chol(H,'lower');
% Linv = inv(Lmat);
% Check if H is positive definite
% if p ~= 0
%     disp('H is not positive definite')
% end
%%

% Compute the linear term of the cost function
fcon = G*(x - xTarget);
% Compute the matrix A and b for the inequality constraint formulation
Acon = -F;
bcon = -(J*x + L*xTarget + bb);


[U,status,iA1]=mpcqpsolver(H,fcon,Acon,bcon,[],zeros(0,1),iA,opt);
u = U(1:m,1);


end