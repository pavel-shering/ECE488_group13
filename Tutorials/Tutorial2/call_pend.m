function [] = call_pend()
    tspan=[0 2*pi]; % set time interval 
    z0=[pi/3,0]; % set initial conditions
    [t,z]=ode23(@pend,tspan,z0);
    plot(t,z(:,1))
    function dzdt = pend(t,z)
        G=9.81; L=2; % set constants
        z1=z(1); % get z1
        z2=z(2); % get z2
        dzdt = [z2 ; -G/L*sin(z1);];
    end
end