function [Dt,Et,bt]=genStageConstraints(A,B,D,cl,ch,ul,uh)
% The function generates the stage constraints for the linear (or
% linearized) system, such that the constraints in the form of 
% 
% x(k+1) = A * x(k) + B * u(k)
% ul <= u(k)  <= uh
% cl <= D * x(k) <= ch
% 
% can be written as 
% 
% Dt * x(k) + Et * u(k) <= bt
% Do not forget to augment the matrices by input constraints!
Dt = [D*A; -D*A; zeros(size(ul,1), size(A,1)); zeros(size(uh,1), size(A,1))];
Et = [D*B; -D*B; -eye(size(ul,1)); eye(size(uh,1))];
bt = [ch; -cl; -ul; uh];

end