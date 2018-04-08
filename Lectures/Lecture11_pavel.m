close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

% Youla Parametarization

s = tf('s');
P = 100 / ( (s+1)*(s-2)*(s+4) );
P_prime = 100 / ( s* (s+1)*(s-2)*(s+4) );

%find controller C(s) that stabilizes it
% plot step response for 10 seconds

%step 1 - get A,B,C,D for the plant
[A, B, C, D] = ssdata(P_prime);

%step 2 - obbtain k - controllability?
p = [-1 -2 -30 -40]
k = place (A, B, p);

%check stability
eig_vals = eig(A-B*k)

%step 3 - obtain f - observability?
p2 = [-50,-60,-700, -800];
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

r = 100 / (s+100)
C = (xp + r*dp) / (yp - r*np)

% feedback system
Tfinal = 10;
sys = feedback(P*C*1/s, 1);
step(sys, Tfinal)
