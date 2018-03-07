clear all; clc; close all;

A = [ -1 0;
       0 -2];
B = [1 1;
     1 0]; 
 
C = [2 0; 
     0 1];
 
D = zeros(2,2);

% [num1 den1] = ss2tf(A,B,C,D,1)
% [num2 den2] = ss2tf(A,B,C,D,2)
% 
% g11 = tf(num1(1,:),den1)
% g12 = tf(num1(2,:),den1)
% 
% g21 = tf(num2(1,:),den2)
% g22 = tf(num2(2,:),den2)
%  
%  % transfer function
% G = [g11 g12;
%      g21 g22]

G = tf(ss(A,B,C,D));
 
% controllable cononical form 
a11 = [ 0  1;
       -2 -3];

a12 = a11;
a21 = a11;
a22 = 0;

b11 = [0; 1];
b12 = b11;
b21 = b11;
b22 = 1;

c11 = [4 2];
c12 = c11;
c21 = [1 1];
c22 = 0;

d11 = 0;
d12 = 0;
d21 = 0;
d22 = 0;
 
A_ctr = blkdiag(a11, a21, a12, a22)

b_prime = [b11;
           b12];
b_prime2 = [b21;
            b22];
B_ctr = blkdiag(b_prime,  b_prime2)

c_prime = blkdiag(c11, c12);
c_prime2 = blkdiag(c21, c22);
C_ctr = [c_prime c_prime2]

D_ctr = [d11 d12;
         d21 d22]

s= tf('s');
G1 = C_ctr*inv(s*eye(7) -A_ctr)*B_ctr+D_ctr

Q_ctr = ctrb(A_ctr, B_ctr)
rank(Q_ctr)

indep_col1 = [1 0 0 0 0 0 6]';
indep_col2 = [0 1 0 0 1 1 6]';
T = [Q_ctr(:,[1:4,6]) indep_col1 indep_col2]
rank(T)


A_bar = inv(T)*A_ctr*T
B_bar = inv(T)*B_ctr
C_bar = C_ctr*T
D_bar = D_ctr

A_bar_prime = A_bar(1:5,1:5)
B_bar_prime = B_bar(1:5,:)
C_bar_prime = C_bar(:,1:5)
D_bar_prime = D_bar

s= tf('s');
G2 = C_bar_prime*inv(s*eye(5) -A_bar_prime)*B_bar_prime+D_bar_prime

%check if the same
G1 - G2
%check controllability
Q_bar = ctrb(A_bar_prime, B_bar_prime)
rank(Q_bar)

%check observabiblity
sys = ss(A_bar_prime, B_bar_prime, C_bar_prime, D_bar_prime)
Ob = obsv(sys);
rank(Ob)


% quizd
% find the rank 
% choose the rank amount of cals and add missing to get full rank 
% circle the input and output amount for controllabiblity 
% then without finding transfer functions do D CB ACB ... (in the notes)
% go up to n due to Cayley Hamiltons Theorem

%% ISOLATE THE OBSERVABLE PART NOW
indep_row1 = [0 0 0 0 1]
indep_row2 = [0 0 0 1 0]
indep_row3 = [0 0 1 0 0]

T_ob = [Ob(1:2,:);
        indep_row1;
        indep_row2;
        indep_row3]
    
rank(T_ob)

A_bar_ob = T_ob*A_bar_prime*inv(T_ob)
B_bar_ob = T_ob*B_bar_prime
C_bar_ob = C_bar_prime*inv(T_ob)
D_bar_ob = D_bar_prime;

A_bar_prime = A_bar_ob(1:2,1:2)
B_bar_prime = B_bar_ob(1:2,:)
C_bar_prime = C_bar_ob(:,1:2)
D_bar_prime = D_bar_ob

% minimalized system that is controllable and observable
sys_min = ss(A_bar_prime, B_bar_prime, C_bar_prime, D_bar_prime)

Q_min = ctrb(sys_min)
rank(Q_min)

R_min = obsv(sys_min)
rank(R_min)