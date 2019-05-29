function [Alin, Blin] = linearizePendulumODE(x, sys, Ts)
    % This function is linearization of the continuous nonlinear model in the form
    % of dx/dt=f(x,u). Approximation takes form 
    % dx/dt = Alin * x + Blin * u
    %
    % Matrices Alin, Blin are Jacobians of f(x,u) about x and u
    % respectively and if the sampling period is provided, these matrices
    % are discretized
    
    % Parameters used to represent constant parameters in dynamical
    % model of the system. 
    g = sys.g;
    l = sys.l;
    b = sys.b;
    
    % Calculate the linearized state & input matrices     
    Alin = [0, 1; 
          (-g/l)*cos(x(1)), -b];
    
    Blin = [0;
            1];
        
    % Convert the continuous system to discrete system if the sampling
    % period is provided
    c_sys = ss(Alin, Blin, [], []);

    if Ts > 0
        d_sys = c2d(c_sys, Ts);
        Alin = d_sys.A;
        Blin = d_sys.B;
    end
    
end
