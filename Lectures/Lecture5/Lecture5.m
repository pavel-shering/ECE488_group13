close all; clear all; clc;

%model variables
syms th thd thdd T
syms x1 x2 u 

x = [x1, x2];

eqn = thdd + thd + sin(th) == T;
eqn = subs(eqn, [th thd, T], [x1, x2 u]);

%equilibrium point for linearization
Eqx = [0 0]; % [pos vel]
u0 = 0;

%formulating the state space
y = x(1);
xdot = [x(2) solve(eqn, thdd)].';

A = jacobian(xdot, x.');
B = jacobian(xdot, u);
C = jacobian(y, x.');
D = jacobian(y, u);

%compute at Eq point
A = double(subs(A, [x, u], [Eqx, u0]));
B = double(subs(B, [x, u], [Eqx, u0]));
C = double(subs(C, [x, u], [Eqx, u0]));
D = double(subs(D, [x, u], [Eqx, u0]));

% xdot = A*x + B*u
% y = C*X + D*u

%% Initial condotions
tspan = 0:0.001:10; % set time interval

figure(1)
hold on
% for i = 0:0.1:3
    T = 0;
    X0 = [0.6 0];
%     X0 = [i 0];
    linX0 = [0.6 0];

    %linear derivative
    [t,X] = ode45(@(t,X)lin_pend(t, X, A, B, T), tspan, linX0);
    deltaX = X + Eqx;

    % figure(1)
    % hold on
    plot(t,deltaX(:,1));
    disp([t,deltaX(:,1)]) % displays t and y(t)

    % non-linear derivative
    [t,Xnl] = ode45(@(t,Xnl)nl_pend(t, Xnl, T), tspan, X0);

    plot(t,Xnl(:,1));
    xlabel('time');
    ylabel('position');
    title(['System Response at position = ',num2str(X0(1)), ...
        ' velocity= ', num2str(X0(2))]);
    legend('Linearized', 'Non-Linearized')
    disp([t,Xnl(:,1)]) % displays t and y(t)
% end

% the pi and 0 arent equal due to numerical integration.
