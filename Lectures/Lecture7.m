clc;clear all;close all;


%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta 

% very specific choice of numbers will give you a jordan block
% at some point we will be give a system to design a controller for
% will be designing for some specs, dont put all your polls at the same
% location you can make a jordan block.
A = [1  0  0;
     0  3 -4;
     0  1 -1];
 
 
A = A + normrnd(0, .5)

eig(A)
[v d] = eig(A)

B = inv(v)
C = inv(B)

v-C

