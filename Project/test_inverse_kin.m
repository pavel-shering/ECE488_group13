x = 0:0.1:2; % x coordinates for validation
y = 8:0.1:10; % y coordinates for validation

[X,Y] = meshgrid(x,y)

c2 = (X.^2 + Y.^2 - l1^2 - l2^2)/(2*l1*l2)
s2 = sqrt(1 - c2.^2)
THETA2D = atan2(s2,c2); % theta2 is deduced

k1 = l1 + l2.*c2;
k2 = l2*s2;
THETA1D = atan2(Y,X) - atan2(k2,k1); % theta1 is deduced