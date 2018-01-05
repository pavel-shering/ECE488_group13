clc;clear all;close all;

s = tf('s')
P = [10/((s+1)*s)  1/((s+100)*s);
     1/((s+100)*s)  10/((s+1)*s)];
 

y = feedback(P*C, 1)*U;
% PD controllers
Kp1 = 10;
Kd1 = 1;

Kp2 = 10;
Kd2 = 1;

C1 = Kd1*s + Kp1;
C2 = Kd2*s + Kp2;



C = [C1   0;
      0  C2];

  
  
% %transfer functions
% TF1 = feedback(P*C1,1);
% 
% TF2 = feedback(P*C2,1);

syms y1 y2 u1 u2


y1 = 
