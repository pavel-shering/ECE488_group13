function [T,Y] = call_osc()
    tspan = [0 3000];
    y1_0 = 2;
    y2_0 = 0;
    [T,Y] = ode15s(@osc,tspan,[y1_0 y2_0]);
    plot(T,Y(:,1),'o')
end