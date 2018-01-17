function dydt = osc(t,y)
    dydt = zeros(2,1); % this creates an empty column 3 
    %vector that you can fill with your two derivatives:
    dydt(1) = y(2);
    dydt(2) = 1000*(1 - y(1)^2)*y(2) - y(1);
    %In this case, y(1) is y1 and y(2) is y2, and dydt(1)
    %is dy1/dt and dydt(2) is dy2/dt.
end