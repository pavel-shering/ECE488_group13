close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta


% step 1: b,a,Kp,Kd find those, closed loop poles at -1
s = tf('s');
b = [0.1 0.5 1.0]
a = [3.1 3.5 4.0]
Kp = 1
Kd = [3.31 4.75 7]

% step 2: (Hand this in) look at step response for b = 0.1, 0.5, 1 ... overshoot should be
% greater
figure(1)
hold on
for i = 1:3
    P = 1 / (s*(s-b(i)));
    
    %step 1 - get A,B,C,D for the plant
    [A, B, C, D] = ssdata(P);

    %step 2 - obbtain k - controllability?
    p = [-1 -2]
    k = place (A, B, p);

    %check stability
    eig_vals = eig(A-B*k)

    %step 3 - obtain f - observerservability?
    p2 = [-50,-60];
    f = place(A', C', p2)
    f = f'

    % check if stable 
    eig_vals2 = eig(A-f*C)

    %step 4 - get the xp, yp, dp, np 
    [xp_n xp_d] = ss2tf(A-f*C, -f, -k, 0);
    [np_n np_d] = ss2tf(A-B*k, B, C-D*k, D);
    [yp_n yp_d] = ss2tf(A-f*C, -B+f*D, -k, 1);
    [dp_n dp_d] = ss2tf(A-B*k, B, -k, 1);

    xp = tf(xp_n, xp_d);
    np = tf(np_n, np_d);
    yp = tf(yp_n, yp_d);
    dp = tf(dp_n, dp_d);
    
    C = (Kd(i)*s + Kp) / (s+a(i));
    
    sys= feedback(P*C,1);
    step(sys);
    
end
legendCell = cellstr(num2str(b, 'k=%d '));
legend(legendCell);

% step 3: (for fun) Youla parameterazation and look at the step

% Youla Parametarization
%find controller C(s) that stabilizes it
% plot step response for 10 seconds



% r = 100 / (s+100)
r = 1 / (s+1)
r = 10000 / ((s+200)*(s+10))
% r = 10

% feedback system
Tfinal = 10;
figure(2)
hold on
for i = 1:3
    P = 1 / (s*(s-b(i)));
    
    %step 1 - get A,B,C,D for the plant
    [A, B, C, D] = ssdata(P);

    %step 2 - obbtain k - controllability?
    p = [-1 -2]
    k = place (A, B, p);

    %check stability
    eig_vals = eig(A-B*k)

    %step 3 - obtain f - observerservability?
    p2 = [-50,-60];
    f = place(A', C', p2)
    f = f'

    % check if stable 
    eig_vals2 = eig(A-f*C)

    %step 4 - get the xp, yp, dp, np 
    [xp_n xp_d] = ss2tf(A-f*C, -f, -k, 0);
    [np_n np_d] = ss2tf(A-B*k, B, C-D*k, D);
    [yp_n yp_d] = ss2tf(A-f*C, -B+f*D, -k, 1);
    [dp_n dp_d] = ss2tf(A-B*k, B, -k, 1);

    xp = tf(xp_n, xp_d);
    np = tf(np_n, np_d);
    yp = tf(yp_n, yp_d);
    dp = tf(dp_n, dp_d);
    C = (xp + r*dp) / (yp - r*np);
    
    sys = feedback(P*C,1);
%     sys = minreal(sys)
    step(sys);
    pole(sys)


    
end
legendCell = cellstr(num2str(b, 'k=%d '));
legend(legendCell);


% sys = feedback(P*C, 1);
% step(sys, Tfinal)
