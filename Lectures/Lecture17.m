close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

s = tf('s');


A = [ 0  1;
     -2 -2];

B = [1; 
     0];

p = [-1 -2];

k = place (A, B, p)

% gives an error beacuse they are the same location
% p = [-1 -1]

p = [-1 -1.001]

k = place (A, B, p)

% k = eig(A-B*k)

% create an n = 3 state space system where the poles are initially at -1.
% We want to move the poles to -2. It cannot be in controllable canonical
% form. 

% solution to the problem A-Bk
Asol = [ 0   1  0;
         0   0  1;
        -8 -12 -6]; 

% the starting cononical form
A = [ 0   1  0;
      0   0  1;
     -1  -3 -3]; 
B = [0 0 1]'
% A-Bk will provide the solution (Asol)
k = [7 9 3]

Acheck = A - B*k
%should be zero
Acheck - Asol

% simularity transformation matrix
T = rand(3,3)
rank(T) % has to be rank 3

% find A_prime
A_prime = T*A*inv(T)
rank(A_prime)
A_prime = round(A_prime)
rank(A_prime)

B_prime = T*B
B_prime = round(B_prime)

%% SOLUTION CHECK
p = [-1.0000001 -1.0000005 -1.0000004];
k = place(A_prime, B_prime, p)

eig(A_prime -  B_prime*k)



