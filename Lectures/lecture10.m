close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

n1= 0.5;
d1= [1 2 4];
g_s = tf(n1,d1); % transfer function g_s= 0.5/s^2+2s+4

figure(1)
delta_1 = 0.9; 
sys_1 = feedback(g_s,delta_1);
nyquist1(sys_1);
%step(sys_1);

figure(2)
alpha = 0.1;
s= tf('s');
delta_2 = (0.9*alpha)/(s +alpha)
sys_2 = feedback(g_s,delta_2);
nyquist1(sys_2);
%step(sys_2);

figure(3)
t_delay = 0.1;
delta_3 = 0.9*exp(-s*t_delay);
sys_3 = feedback(g_s,delta_3);
%step(sys_3);
nyquist(sys_3);


