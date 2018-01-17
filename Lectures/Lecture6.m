clc;clear all;close all;

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta 

B = [ 2  1  0 -1;
      1  0 -1 -2;
     -1 -1 -1 -1];

A = [1  0 1 2 3;
     1  1 0 3 3;
     2 -1 3 3 6];
 
Z = null(A,'r')
Zr = orth(A)
D = null(B,'r')
Dr = orth(B)


%% Deliverable
% no solutions
y = [ 2
     -4];
A = [2 0 0
     0 0 0];

disp('no solution')
rank(y)
rank(A)
rank([y A])

% one solution
disp('one solution')
% impossible to get one solution due to the fact that size of A is not rank
% of A
% if rank(A) = n, to have one solution A must be an nxn matrix



% infinit # of solutions
disp('infinite # solution')
y = [0 
     5];
A = [5 -10 0;
     0 -15 0];
     
rank(y)
rank(A)
rank([y A])

     