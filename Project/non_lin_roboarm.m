function Xdot = non_lin_roboarm(t, Xnl, T, g, m1, m2, l1, l2, c1, c2)
    Xdot = [ Xnl(2);
            (3*(400*l2*T(1) - 400*l2*T(2) - 600*l1*T(2)*cos(Xnl(3)) - 400*c1*l2*Xnl(2) + 400*c2*l2*Xnl(4) - 1962*l1*l2*m1*cos(Xnl(1)) - 3924*l1*l2*m2*cos(Xnl(1)) + 600*c2*l1*Xnl(4)*cos(Xnl(3)) + 2943*l1*l2*m2*cos(Xnl(1) + Xnl(3))*cos(Xnl(3)) + 200*l1*l2^2*m2*Xnl(2)^2*sin(Xnl(3)) + 200*l1*l2^2*m2*Xnl(4)^2*sin(Xnl(3)) + 300*l1^2*l2*m2*Xnl(2)^2*cos(Xnl(3))*sin(Xnl(3)) + 400*l1*l2^2*m2*Xnl(2)*Xnl(4)*sin(Xnl(3))))/(100*(4*l1^2*l2*m1 + 12*l1^2*l2*m2 - 9*l1^2*l2*m2*cos(Xnl(3))^2));
             Xnl(4);
            -(3*(400*l2^2*m2*T(1) - 1200*l1^2*m2*T(2) - 400*l1^2*m1*T(2) - 400*l2^2*m2*T(2) + 5886*l1^2*l2*m2^2*cos(Xnl(1) + Xnl(3)) - 3924*l1*l2^2*m2^2*cos(Xnl(1)) - 400*c1*l2^2*m2*Xnl(2) + 400*c2*l1^2*m1*Xnl(4) + 1200*c2*l1^2*m2*Xnl(4) + 400*c2*l2^2*m2*Xnl(4) - 5886*l1^2*l2*m2^2*cos(Xnl(1))*cos(Xnl(3)) + 1962*l1^2*l2*m1*m2*cos(Xnl(1) + Xnl(3)) + 200*l1*l2^3*m2^2*Xnl(2)^2*sin(Xnl(3)) + 600*l1^3*l2*m2^2*Xnl(2)^2*sin(Xnl(3)) + 200*l1*l2^3*m2^2*Xnl(4)^2*sin(Xnl(3)) - 1962*l1*l2^2*m1*m2*cos(Xnl(1)) + 2943*l1*l2^2*m2^2*cos(Xnl(1) + Xnl(3))*cos(Xnl(3)) + 600*l1*l2*m2*T(1)*cos(Xnl(3)) - 1200*l1*l2*m2*T(2)*cos(Xnl(3)) - 2943*l1^2*l2*m1*m2*cos(Xnl(1))*cos(Xnl(3)) + 200*l1^3*l2*m1*m2*Xnl(2)^2*sin(Xnl(3)) + 400*l1*l2^3*m2^2*Xnl(2)*Xnl(4)*sin(Xnl(3)) - 600*c1*l1*l2*m2*Xnl(2)*cos(Xnl(3)) + 1200*c2*l1*l2*m2*Xnl(4)*cos(Xnl(3)) + 600*l1^2*l2^2*m2^2*Xnl(2)^2*cos(Xnl(3))*sin(Xnl(3)) + 300*l1^2*l2^2*m2^2*Xnl(4)^2*cos(Xnl(3))*sin(Xnl(3)) + 600*l1^2*l2^2*m2^2*Xnl(2)*Xnl(4)*cos(Xnl(3))*sin(Xnl(3))))/(100*(12*l1^2*l2^2*m2^2 + 4*l1^2*l2^2*m1*m2 - 9*l1^2*l2^2*m2^2*cos(Xnl(3))^2))];
%     if(Xdot(2) < 1e-12)
%         Xdot(2) = 0;
%     end
%     if(Xdot(4) < 1e-12)
%         Xdot(4) = 0;
%     end
end