clc;clear all;close all;

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta 

Ess = 0.5; % steady state errorfor ramp 
PM = 60; % [deg] desired phase margin

% plant
s = tf('s');
P = 1/((s+1)*(s+5)*(s+10));

% system
k = 320;
G = k*P;

figure(1) 
hold on
margin(G);
[Gm,Pm,Wgm,Wpm] = margin(G);

%% looping to iteratively find alpha that converges
% to design a different controller, change the gain k and the aiming PM as
% well as the iteration loop value
for i = 1:10
    i
    phi_max = PM - Pm;
    a = ( 1 + sind(phi_max) ) / ( 1 - sind(phi_max) )
    gain = 10*log10(a);
    wc = getGainCrossover(G,gain);
    t1 = 1/(sqrt(a)*wc);
    z1 = 1/(a*t1);
%     t2 = a^2 * t1 / 0.1
    t2 = Ess*k / 50;
    z2 = a/t2;
    td= t1;
    C = k * ( (s + z1) * ( s + z2) ) / ( (s + 1/td) * s );

    G = C*P;
    [Gm,Pm,Wgm,Wpm] = margin(G);
    margin(G);
end

%% verifying that the steady state error is less than 0.5 by zooming into
% the 300s time zone and calculating error, which is around 0.17
figure(2)
hold on
sys = feedback(G,1);
% [y,t] = step(sys*1/s)
step(sys*1/s)
step(1/s)
title('Ramp Response');

figure(3)
hold on
step(sys*1/s)
step(1/s)
axis([280,281,280,281])
title('Ramp Response - Steady State Error Analysis Zoomed');

%% analysing the effect of varying k of the controller
figure(4)
hold on
C = ( (s + z1) * ( s + z2) ) / ( (s + 1/td) * s );
G = k * C*P;
[Gm,Pm,Wgm,Wpm] = margin(G);
margin(G);
k_leg = [k];
for i = 1:5
    k = k + i*100
    G = k*C*P;
    [Gm,Pm,Wgm,Wpm] = margin(G);
    margin(G);
    k_leg = [k_leg ; k];
end 
title('Effects of Varying k on Phase Margin');
legendCell = cellstr(num2str(k_leg, 'k=%-d'));
legend(legendCell);

% large PM makes system more robust
    % large PM is safety margin to noise in the system
    % stabilty
% thus increasing k decreases PM 