function dxdt = genPendulumODE(x,u,sys)
    % This function represents nonlinear model of simple mathematical
    % pendulum in medium with linear viscocity 
    g = sys.g;
    l = sys.l;
    b= sys.b;
    
    dxdt =zeros(2,1);
    dxdt(1) = x(2);
    dxdt(2) = (-g/l) * sin(x(1)) -b * x(2) + u;
end