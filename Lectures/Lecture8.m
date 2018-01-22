clc; clear all; close all;

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta 

A = [-2  1;
      0 -2];
  
B = [0 1].';
C = [1 1];

x = [0 0].';
u = 1;


% xdot = Ax + Bu
% y = Cx + Du

t = 0: 0.01: 10;

delta_t=0.1; % step size
time = 0:delta_t:5;                                 
y = zeros(1, 5/delta_t);
y_analytical = zeros(1, 5/delta_t);
for t=1:(length(time)-1) % calculation loop
    k_1 = A*x + B*u;
    k_2 = A*(x+0.5*delta_t*k_1)+B*u;
    k_3 = A*(x+0.5*delta_t*k_2)+B*u;
    k_4 = A*(x+delta_t*k_3)+B*u;
    x = x + (1/6)*(k_1+2*k_2+2*k_3+k_4)*delta_t;  % main equation
    y(t) = C*x;
    y_analytical(t) = 3/4 -(time(t)/2)*exp(-2*time(t)) - (3/4)*exp(-2*time(t));
end

figure(1);
hold on;
plot(y)
plot(y_analytical)
legend('runge kutta', 'analytical');

