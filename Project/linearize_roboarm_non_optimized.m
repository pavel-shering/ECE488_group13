function [A B C D U] = linearize_roboarm_non_optimized(A,B,C,D,U, Eqm_point)
   
    syms x1 x2 x3 x4 u1 u2
    %% sub in eqm_point
    rx1 = Eqm_point(1);
    rx2 = Eqm_point(2);
    rx3 = Eqm_point(3);
    rx4 = Eqm_point(4);
    
    u1r = double(subs(U(1), [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]))
    u2r = double(subs(U(2), [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]))
    % must subb in the input torques.... i think

    %% should not have any variables here!!!
    A = double(subs(A, [x1 x2 x3 x4 u1 u2], [rx1, rx2, rx3, rx4, u1r, u2r]));
    B = double(subs(B, [x1 x2 x3 x4 u1 u2], [rx1, rx2, rx3, rx4, u1r, u2r]));
    C = double(subs(C, [x1 x2 x3 x4 u1 u2], [rx1, rx2, rx3, rx4, u1r, u2r]));
    D = double(subs(D, [x1 x2 x3 x4 u1 u2], [rx1, rx2, rx3, rx4, u1r, u2r]));

    U = [u1r u2r];
end