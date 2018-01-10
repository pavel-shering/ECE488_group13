clc;clear all;close all;

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta 

s = tf('s')
P = 1/((s+1)*(s+5)*(s+10))

% steady state error
Ess = 0.5; % for ramp 
PM = 55; % [deg]

k = 300 
G = k*P;

margin(G);

[Gm,Pm,Wgm,Wpm] = margin(G)
% solve for a greater buffer than 45, try 50
phi_max =  50 -  Pm

% solve for alpha
a = ( 1 + sind(phi_max) )/ ( 1 - sind(phi_max) ) 

gain = 10*log10(a)
% get the frequency at the magnitute related to alpha
wc = getGainCrossover(G,gain)

[mag,phase,wout] = bode(G,wc)
% check phase
phi_max = - 135 -  phase

t1 = 1/(sqrt(a)*wc)
z1 = 1/(a*t1);
t2 = a^2 * t1 / 0.1
z2 = a/t2;
td= t1;

C = k * ( (s + z1) * ( s + z2) ) / ( (s + 1/td) * s )

G = C*P
[Gm,Pm,Wgm,Wpm] = margin(G)
figure(1)
margin(G)

figure(2)
step(G*1/s, 10000000)

% k = 280
% C = k * ( (s + z1) * ( s + z2) ) / ( (s + 1/td) * s )
% 
% G = C*P
% [Gm,Pm,Wgm,Wpm] = margin(G)
% figure(2)
% margin(G)
% 
% 
% 
% k = 260
% C = k * ( (s + z1) * ( s + z2) ) / ( (s + 1/td) * s )
% 
% G = C*P
% [Gm,Pm,Wgm,Wpm] = margin(G)
% figure(2)
% margin(G)

% large PM makes system more robust
    % large PM is safety margin to noise in the system
    % stabilty
% increasing k decreases PM 
    % (same as decreasing K increases PM)
    

% second Controller

%{

s = tf('s')
P = 1/((s+1)*(s+5)*(s+10))

% steady state error
Ess = 0.5; % for ramp
PM = 45; % [deg]

k = 800
G = k* P;

margin(G);

%[Gm,Pm,Wgm,Wpm] = margin(G)

%phi_max = 45 -  Pm

%phi max is the difference between what we have and what we need. How much
%to get to -135

phi_max = degtorad(40)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

phi_max=degtorad(11)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

phi_max=degtorad(30)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

phi_max=degtorad(21)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

phi_max=degtorad(26)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

phi_max=degtorad(23)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

phi_max=degtorad(25)

a = ( 1 + sin(phi_max) )/ ( 1 - sin(phi_max) )

gain = 10*log10(a)

%}


% Alpha after the above commented iterations led to a gain at 3.9163 which
% on the bode plot was wc of 5.72 rad/s. The entire process started off
% with a gain of K = 800 and that was used in the graph during the
% iterations

% When K = 10, the phase margin is 92.7 degrees
% When K = 200, the phase margin is 100 degrees
% When K = 400, the phase margin is   73   degrees
% When K = 600, the phase margin is   55.2   degrees
% When K = 700, the phase margin is   48.3   degrees. This is the final
% second controller
% When K = 800, the phase margin is   42.4   degrees. Much like in the
% compensated bode plot from the section 1 lecture notes that has phase
% margin of 42.6 (3rd last page of section 1)


s= tf('s')
P = 1/((s+1)*(s+5)*(s+10))
k=700
a = 2.4639
wc = 5.72
t1 = 1/(sqrt(a)*wc)
z1 = 1/(a*t1);
t2 = a^2 * t1 / 0.1;
z2 = a/t2;
td= t1;

C = k * ( (s + z1) * ( s + z2) ) / ( (s + 1/td) * s )

G = C*P
[Gm,Pm,Wgm,Wpm] = margin(G)
figure(3)
margin(G)
