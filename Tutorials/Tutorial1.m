clc;clear all;close all;

s = tf('s')
P = 10/(s^2+10*s)
figure(1)
bode(P)

%crit damped
Kp = 10;
Kd = 1;

C = Kd*s + Kp;
TF = feedback(P*C, 1);
figure(2)
subplot(3,1,1)
step(TF)
title('crit damped')


%over damped
Kp = 10;
Kd = 10;

C = Kd*s + Kp;
TF = feedback(P*C, 1);
subplot(3,1,2)
step(TF)
title('over damped')


%under damped
Kp = 100;
Kd = 1;

C = Kd*s + Kp;
TF = feedback(P*C, 1);
subplot(3,1,3)
step(TF)
title('under damped')

