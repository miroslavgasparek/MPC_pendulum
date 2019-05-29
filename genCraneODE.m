function [A,B,C,D] = genCraneODE(m,M,MR,r,g,Tx,Ty,Vm,Ts)
% Inputs:
% m = Pendulum mass (kg)
% M = Cart mass (kg)
% MR = Rail mass (kg)
% r = String length (m)
% g = gravitational accelaration (m*s^-2)
% Tx = Damping coefficient in X direction (N*s*m^-1)
% Ty = Damping coefficient in Y direction (N*s*m^-1)
% Vm = Input multiplier (scalar)
% Ts = Sample time of the discrete-time system (s)
% Outputs:
% A,B,C,D = State Space matrices of a discrete-time or continuous-time state space model

% The motors in use on the gantry crane are identical and therefore Vx=Vy.
Vx=Vm;
Vy=Vm;

% replace A,B,C,D with the correct values
A = zeros(8,8);
B = zeros(8,2);
C = zeros(8,8);

A=[0, 1, 0, 0, 0, 0, 0, 0;
   0, -Tx/(M+MR), 0, 0, (g*m)/(M+MR), 0, 0, 0;
   0, 0, 0, 1, 0, 0, 0, 0;
   0, 0, 0, -Ty/M, 0, 0, (g*m)/M, 0;
   0, 0, 0, 0, 0, 1, 0, 0;
   0, Tx/(r*(M+MR)), 0, 0, -(g*(M + MR + m))/(r*(M + MR)), 0, 0, 0;
   0, 0, 0, 0, 0, 0, 0, 1;
   0, 0, 0, Ty/(M*r), 0, 0, -g*(M+m)/(M*r), 0];

B=[0, 0;
   Vx/(M+MR), 0;
   0, 0;
   0, Vy/M;
   0, 0;
   -Vx/(r*(M+MR)), 0;
   0, 0;
   0, -Vy/(M*r)];

C=eye(8);

D=zeros(8,2);

% if Ts>0 then sample the model with a zero-order hold (piecewise constant) input, otherwise return a continuous-time model
if Ts>0
    system = ss(A,B,C,D);
    system_d = c2d(system, Ts);
    
    A = system_d.A;
    B = system_d.B;
    C = system_d.C;
    D = system_d.D;
end

end