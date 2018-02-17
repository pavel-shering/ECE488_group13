function [A B C D u1 u2] = linearize_roboarm_non_optimized(A,B,C,D, T1, T2, Eqm_point)
   
    syms x1 x2 x3 x4
    %% sub in eqm_point
    rx1 = Eqm_point(1);
    rx2 = Eqm_point(2);
    rx3 = Eqm_point(3);
    rx4 = Eqm_point(4);
    
    %% should not have any variables here!!!
    A = double(subs(A, [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]));
    B = double(subs(B, [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]));
    C = double(subs(C, [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]));
    D = double(subs(D, [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]));
    u1 = double(subs(T1, [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]));
    u2 = double(subs(T2, [x1 x2 x3 x4], [rx1, rx2, rx3, rx4]));
    
end