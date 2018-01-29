close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

dt = 0.01;
Tf = 15;
t = 0:dt:Tf;


zeta = 0.01;
harm = 10;

Kp = harm^2;
Kd = 2*zeta*sqrt(Kp)-0.1;

s = tf('s');
P = 1 / (s * (s + 0.1));

% choose Kp and Kd to have different peaks in S
% Kd = 0.9;
% Kp = 100;
C = Kd*s + Kp;

% plot S and T, find poor tracking for some types of sine input.
% plant and controller
G = feedback(P*C, 1);

% sensitivity function S
S = 1 / (1 + P*C);

% complementatry sensitivity T 
T = (P*C) / (1 + P*C);

figure(1)
bode(S, T);
% subplot(2,1,2);
% bode(T)

figure(2)
subplot(2,1,1);
lsim(G, sin(10*t), t);
subplot(2,1,2);
lsim(G, sin(0.1*t), t);




