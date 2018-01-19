function xdot = lin_roboarm(t, X, A, B, T)
    xdot = A*X + B*T;
end