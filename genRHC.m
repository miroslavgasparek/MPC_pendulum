function K = genRHC(H,G,m)
% H and G are the cost function matrices
% m is the number of control inputs
% K is the RHC law gain

% your code here
% Calculate L
L = (H\G);
% Calculate the RHC gain matrix
K = - L(1:m,:);

end