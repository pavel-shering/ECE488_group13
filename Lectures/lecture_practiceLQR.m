clear all; clc; close all;


A = [ 0 1 0;
      0 0 1;
      3 3 1];
  
B = [ 0 0 1 ]';

Q = diag( [2 1 1]);

R = 1;

haml = [ A -B*inv(R)*B';
        -Q -A']
    
[V D] = eig(haml)

V = V(:,[2,3,4])

V1 = V([1:3],[1:3])
V2 = V([4:6],[1:3])

P = V2*inv(V1)

% check to see that its positive semi definate 
eig(P) % all are greater than 0

k = inv(R)*B'*P 
eig(A - B*k)

% validate
[k P ev] = lqr(A,B,Q,R)

