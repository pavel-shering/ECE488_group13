function [A B C D u1 u2] = linearize_roboarm(Eqm_point)
    x1 = Eqm_point(1);
    x2 = Eqm_point(2);
    x3 = Eqm_point(3);
    x4 = Eqm_point(4);
    
    % this is steady state torque after parameters are subbed in
    u1 = (981*cos(x1 + x3))/200 + (2943*cos(x1))/200;
    u2 = (981*cos(x1 + x3))/200;  
    
    % jacobians with subbed in params
    A = [                                                                       0,                                                                                                                      1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                     0;
    -(3*((2943*sin(x1))/50 - (2943*sin(x1 + x3)*cos(x3))/100))/(9*cos(x3)^2 - 16),                                      -(3*(4*x2*sin(x3) + 4*x4*sin(x3) + 6*x2*cos(x3)*sin(x3) - 20))/(9*cos(x3)^2 - 16),                                                                                                                                                                                                                                                                                                                - (3*(2*x2^2*cos(x3) + 2*x4^2*cos(x3) + 3*x2^2*cos(x3)^2 - 3*x2^2*sin(x3)^2 + 6*u2*sin(x3) - 30*x4*sin(x3) - (2943*cos(x1 + x3)*sin(x3))/100 - (2943*sin(x1 + x3)*cos(x3))/100 + 4*x2*x4*cos(x3)))/(9*cos(x3)^2 - 16) - (54*cos(x3)*sin(x3)*(4*u1 - 4*u2 - 20*x2 + 20*x4 - (2943*cos(x1))/50 + 2*x2^2*sin(x3) + 2*x4^2*sin(x3) - 6*u2*cos(x3) + 30*x4*cos(x3) + (2943*cos(x1 + x3)*cos(x3))/100 + 4*x2*x4*sin(x3) + 3*x2^2*cos(x3)*sin(x3)))/(9*cos(x3)^2 - 16)^2,                                               -(3*(30*cos(x3) + 4*x2*sin(x3) + 4*x4*sin(x3) + 20))/(9*cos(x3)^2 - 16);
                                                                                0,                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                     1;
 -(3*((1962*sin(x1 + x3))/25 - (2943*sin(x1))/50 - (8829*cos(x3)*sin(x1))/100 + (2943*sin(x1 + x3)*cos(x3))/100))/(9*cos(x3)^2 - 16), (3*(20*x2*sin(x3) - 30*cos(x3) + 4*x4*sin(x3) + 12*x2*cos(x3)*sin(x3) + 6*x4*cos(x3)*sin(x3) - 20))/(9*cos(x3)^2 - 16), (3*(10*x2^2*cos(x3) - (1962*sin(x1 + x3))/25 + 2*x4^2*cos(x3) + (8829*cos(x1)*sin(x3))/100 + 6*x2^2*cos(x3)^2 + 3*x4^2*cos(x3)^2 - 6*x2^2*sin(x3)^2 - 3*x4^2*sin(x3)^2 - 6*u1*sin(x3) + 12*u2*sin(x3) + 30*x2*sin(x3) - 60*x4*sin(x3) - (2943*cos(x1 + x3)*sin(x3))/100 - (2943*sin(x1 + x3)*cos(x3))/100 + 4*x2*x4*cos(x3) + 6*x2*x4*cos(x3)^2 - 6*x2*x4*sin(x3)^2))/(9*cos(x3)^2 - 16) + (54*cos(x3)*sin(x3)*(4*u1 - 20*u2 - 20*x2 + 100*x4 + (1962*cos(x1 + x3))/25 - (2943*cos(x1))/50 - (8829*cos(x1)*cos(x3))/100 + 10*x2^2*sin(x3) + 2*x4^2*sin(x3) + 6*u1*cos(x3) - 12*u2*cos(x3) - 30*x2*cos(x3) + 60*x4*cos(x3) + (2943*cos(x1 + x3)*cos(x3))/100 + 4*x2*x4*sin(x3) + 6*x2^2*cos(x3)*sin(x3) + 3*x4^2*cos(x3)*sin(x3) + 6*x2*x4*cos(x3)*sin(x3)))/(9*cos(x3)^2 - 16)^2, (3*(60*cos(x3) + 4*x2*sin(x3) + 4*x4*sin(x3) + 6*x2*cos(x3)*sin(x3) + 6*x4*cos(x3)*sin(x3) + 100))/(9*cos(x3)^2 - 16)];

    B = [                              0,                                         0;
                  -12/(9*cos(x3)^2 - 16),    (3*(6*cos(x3) + 4))/(9*cos(x3)^2 - 16);
                                       0,                                         0;
   (3*(6*cos(x3) + 4))/(9*cos(x3)^2 - 16), -(3*(12*cos(x3) + 20))/(9*cos(x3)^2 - 16)];
    
    C = [ 1, 0, 0, 0;
          0, 0, 1, 0];
      
    D = [ 0, 0;
          0, 0];
end