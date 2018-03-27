% solve all angles
% x = 8:0.01:21;
% y = 8:0.01:21;

l1 = 14.85
l2 = 14.85

A = [10,20]
B = [20,20]
C = [20,10]
D = [10,10]

X = A(1)
Y = A(2)

% X = B(1)
% Y = B(2)

c2 = (X^2 + Y^2 - l1^2 - l2^2)/(2*l1*l2);
s2 = sqrt(1 - c2^2);
THETA2D = atan2(s2,c2) % theta2 is deduced

k1 = l1 + l2*c2;
k2 = l2*s2;
THETA1D = atan2(Y,X) - atan2(k2,k1) % theta1 is deduced

X_pred = l1 * cos(THETA1D) + l2 * cos(THETA1D + THETA2D); % compute x coordinates
Y_pred = l1 * sin(THETA1D) + l2 * sin(THETA1D + THETA2D);

% [X,Y] = meshgrid(x,y);

% c2 = (X.^2 + Y.^2 - l1^2 - l2^2)/(2*l1*l2);
% s2 = sqrt(1 - c2.^2);
% THETA2D = atan2(s2,c2); % theta2 is deduced
% 
% k1 = l1 + l2.*c2;
% k2 = l2*s2;
% THETA1D = atan2(Y,X) - atan2(k2,k1); % theta1 is deduced
% 
% % q2 = acos(X.^2 + Y.^2 - l1^2 - l2^2)/(2*l1*l2)
% % c2 = cos(q2)
% % s2 = sin(q2)
% % q1 = atan(Y ./ sqrt(l1^2+l2^2+2*l1*l2*c2-Y.^2)) - atan(l2*s2/ (l1+l2.*c2))
% plot(X(:),Y(:),'r.');
% axis equal;
% xlabel('X','fontsize',10)
% ylabel('Y','fontsize',10)
% %   title('X-Y coordinates generated for all theta1 and theta2 ..
% %combinations using forward kinematics formula','fontsize',10)
