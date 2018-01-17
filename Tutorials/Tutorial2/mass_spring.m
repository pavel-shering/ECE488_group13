function [t,Xdot] = mass_spring(M,B,K)
    tspan = 0:0.001:10; % set time interval
    X0 = [0 0]'; % set initial condition
    % dstate evaluates r.h.s. of the ode 

    if ~exist('M','var')
          M = 1;
    end
    if ~exist('B','var')
          B = 1;
    end
    if ~exist('K','var')
          K = 1;
    end
    
    [t,X] = ode45(@(t,X)dstate(t, X, M, B, K), tspan, X0);
    
    plot(t,X(:,1))
    xlabel('time')
    ylabel('position')
    title('position over time for mass spring damper')
    disp([t,X(:,1)]) % displays t and y(t)
    
    function Xdot = dstate(t, X, M, B, K)
        if t < 0.2
            U = 1;
        else 
            U = 0;
        end

        Xdot(1) = X(2);
        Xdot(2) = U/M - B/M * X(2) - K/M*X(1);
        Xdot = Xdot.';
    end
end