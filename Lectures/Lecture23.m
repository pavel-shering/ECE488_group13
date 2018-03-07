close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

s = tf('s');

G = [ 1/(s*(s+1))  1/(s*(s+10));
      1/(s*(s+10)) 1/(s*(s+1)) ];
sys = ss(G)
TFG = tf(sys);


a11 = [ 0  1;
        0 -1];
a12 = [ 0   1;
        0 -10];
a21 = [ 0   1;
        0 -10];
a22 = [ 0  1;
        0 -1];
    
b = [0;
     1];
c = [1 0];

A = blkdiag(a11, a21, a12, a22)
b_prime = [b;
           b];
B = blkdiag(b_prime,  b_prime)

C = blkdiag(c, c);
C = [C C]
D = zeros(2)


% check controllabiblity matrix
sys_og = ss(A,B,C,D)
Q = ctrb(sys_og)
size(Q)
rank(Q)

col = orth(Q)

col_other = rand(8,2)

T = [col col_other];

A_bar = inv(T)*A*T
B_bar = inv(T)*B
C_bar = C*T
D_bar = D

n = 6
A_bar_prime = A_bar(1:n,1:n)
B_bar_prime = B_bar(1:n,:)
C_bar_prime = C_bar(:,1:n)
D_bar_prime = D_bar

sys_cntr = ss(A_bar_prime, B_bar_prime,C_bar_prime,D_bar_prime)
G_check = tf(sys_cntr)


%% CHECK
[n m] = size(A)
for i = 0:m-1
   i
   eq = C*A^i*B - C_bar_prime*A_bar_prime^i*B_bar_prime
end

figure()
step(TFG-G_check)

% observability extractability
R = obsv(sys_cntr)
rank(R)
% rank is full rank thus the matrix is the minimal realization

%% PROBLEM 4
A = [ -0.5 -1 -1.5 -2;
       0    1 -2   -2;
      -0.5 -1  0.5  2;
       0.5  1  0.5 -1];
   
B = [0.5 1 -0.5 0.5]';

C = [1 3 0 -1];

D = 0;

% check controllability
sys = ss(A,B,C,D)
Q = ctrb(sys)
rank(Q)

col = orth(Q)
T = [col rand(4,1)]
rank(T)


A_bar = inv(T)*A*T
B_bar = inv(T)*B
C_bar = C*T
D_bar = D

n = 3
A_bar_prime = A_bar(1:n,1:n)
B_bar_prime = B_bar(1:n,:)
C_bar_prime = C_bar(:,1:n)
D_bar_prime = D_bar

sys_cntr = ss(A_bar_prime, B_bar_prime,C_bar_prime,D_bar_prime)
G_check = tf(sys_cntr)

%% CHECK
[n m] = size(A)
for i = 0:m-1
   eq = C*A^i*B - C_bar_prime*A_bar_prime^i*B_bar_prime
end

% check observability
R = obsv(sys_cntr)
rank(R)

row = orth(R')'

T_ob = [row;
     rand(1,3)];

rank(T_ob)

A_bar_ob = T_ob*A_bar_prime*inv(T_ob)
B_bar_ob = T_ob*B_bar_prime
C_bar_ob = C_bar_prime*inv(T_ob)
D_bar_ob = D_bar_prime;

A_bar_prime = A_bar_ob(1:2,1:2)
B_bar_prime = B_bar_ob(1:2,:)
C_bar_prime = C_bar_ob(:,1:2)
D_bar_prime = D_bar_ob

sys_min = ss(A_bar_prime, B_bar_prime, C_bar_prime, D_bar_prime)

%% CHECK THAT ITS MIN REAL 
Q_min = ctrb(sys_min)
rank(Q_min)

R_min = obsv(sys_min)
rank(R_min)

G_sys_min = tf(sys_min)

step(TFG - G_sys_min)

[n m] = size(A)
for i = 0:m-1
   eq = C*A^i*B - C_bar_prime*A_bar_prime^i*B_bar_prime
end


