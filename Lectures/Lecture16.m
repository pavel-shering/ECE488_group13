close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

s = tf('s');


G = [ 1/(s*(s+1))  1/(s*(s+10));
      1/(s*(s+10)) 1/(s*(s+1)) ];
sys = ss(G)
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

G = C*inv(s*eye(8) -A)*B+D
% minreal(G)


% lecture 17
Co = ctrb(A,B)
rank(Co)
