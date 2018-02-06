close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

s = tf('s');

Td = linspace(0.01, 0.1, 20);

P = 1 / (s*(s+10));

 
Kp = 100;
Kd = 2*sqrt(Kp) - 10;

C = Kd*s + Kp;

figure();hold on;
legendCell = cell(length(Td),1);
for i = 1:length(Td)
    delay = (1 - Td(i) * s / 2) / (1 + Td(i) * s / 2);
    L = C*delay*P;
    sys = feedback(L,1);
    step(sys);
    legendCell{i} = strcat('Td = ',num2str(Td(i)));
end 

title('Increase in Td effect on the undershoot');
legend(legendCell);

figure();
hold on
title('Sensitivity Analysis');
margin(1 / (1 + L));
margin(sys);
legend('Sensitivity S','Complentary Sensitivity T');
