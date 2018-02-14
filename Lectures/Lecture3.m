clc;clear all;close all;

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta 

s = tf('s')
P = 10/(s*(s+2))

%doesnt work all that well, since its not the same as the lecture notes
nyquist(P)
% can also make the s pole be epsilon.
%works ok
figure()
nyquist1(P)

% bode will give you the same information about the phase margin and gain
% margin
figure()
bode(P)
figure()
margin(P)

TF = 2/((s)*(s^3+3*s^2+6*s+1));
k = linspace(0,1.2,100);

% doesnt work gives gain of 1.01 
figure()
rlocus(TF)

% try creating a vector for k to increase the resolution of the linear
% interpolation for rlocus function
figure()
rlocus(TF, k)
