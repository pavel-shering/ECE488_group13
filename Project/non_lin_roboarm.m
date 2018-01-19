function Xdot = non_lin_roboarm(t, Xnl, T)
    Xdot = [ Xnl(2);
            -(3*(80*T(1) - 80*T(2) - 40*Xnl(2) + 40*Xnl(4) - 5886*cos(Xnl(1)) + 200*Xnl(2)^2*sin(Xnl(3)) + 200*Xnl(4)^2*sin(Xnl(3)) - 120*T(2)*cos(Xnl(3)) + 60*Xnl(4)*cos(Xnl(3)) + 2943*cos(Xnl(1) + Xnl(3))*cos(Xnl(3)) + 400*Xnl(2)*Xnl(4)*sin(Xnl(3)) + 300*Xnl(2)^2*cos(Xnl(3))*sin(Xnl(3))))/(100*(9*cos(Xnl(3))^2 - 16));
             Xnl(4);
            (3*(80*T(1) - 400*T(2) - 40*Xnl(2) + 200*Xnl(4) + 7848*cos(Xnl(1) + Xnl(3)) - 5886*cos(Xnl(1)) - 8829*cos(Xnl(1))*cos(Xnl(3)) + 1000*Xnl(2)^2*sin(Xnl(3)) + 200*Xnl(4)^2*sin(Xnl(3)) + 120*T(1)*cos(Xnl(3)) - 240*T(2)*cos(Xnl(3)) - 60*Xnl(2)*cos(Xnl(3)) + 120*Xnl(4)*cos(Xnl(3)) + 2943*cos(Xnl(1) + Xnl(3))*cos(Xnl(3)) + 400*Xnl(2)*Xnl(4)*sin(Xnl(3)) + 600*Xnl(2)^2*cos(Xnl(3))*sin(Xnl(3)) + 300*Xnl(4)^2*cos(Xnl(3))*sin(Xnl(3)) + 600*Xnl(2)*Xnl(4)*cos(Xnl(3))*sin(Xnl(3))))/(100*(9*cos(Xnl(3))^2 - 16))];
end