function Xdot = nl_pend(t, Xnl, T)
    Xdot = [Xnl(2)
            T - Xnl(2) - sin(Xnl(1))];
end