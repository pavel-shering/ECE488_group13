close all; clear all; clc 

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

s = tf('s');

%1. Desing a controller to place poles at -0.5, -1, -5, assuming no
%coupling 
%2. Look at Bode and step response with the coupling in 
%3. Design the low frequency coupling matrix W. Then C' = W
%4. Redo the Bode and Step
Kp = [0.25 1 25];
Kd = [(2*sqrt(Kp(1)) - 1) (2*sqrt(Kp(2)) - 1) (2*sqrt(Kp(3)) - 1)];

P = [ 1/(s*(s+1))  1/(s*(s+10));
      1/(s*(s+10)) 1/(s*(s+1)) ];
  
P_prime = [ 1/(s*(s+1))        0;
                0         1/(s*(s+1)) ];

W = [100/99 -10/99;
     -10/99 100/99];
 

for i = 1:length(Kp)
    
    C = [ Kp(i) + Kd(i)*s/(0.01*s+1)            0;
              0             Kp(i) + Kd(i)*s/(0.01*s+1)];
          
    sys = feedback(P*C, eye(2));
    figure()
    bode(sys)
    hold on;
    
    sys2 = feedback(P*W*C, eye(2));
    bode(sys2)
    
    legendCell = cell(2,1);
    legendCell{1} = strcat('Pole = ',num2str(sqrt(Kp(i))));
    legendCell{2} = strcat('Pole_w = ',num2str(sqrt(Kp(i))));
    title_string = strcat('Bode Plot Effect of -',num2str(sqrt(Kp(i))));
    title_string = [title_string ' pole'];
    title(title_string);
    legend(legendCell);
    
    figure()
    step(sys)
    hold on;
    step(sys2)

    title_string = strcat('Step Response Effect of -',num2str(sqrt(Kp(i))));
    title_string = [title_string ' pole'];
    title(title_string);
    legend(legendCell);
end