clc;
clear all;
close all;

%Student Numbers
% 20523043 Pavel 
% 20509464 Tyler
% 20482444 Henrietta

s = tf('s');
P = 100/((s+1)*(s-2)*(s+4));
[a b c d] = ssdata(P);

k = place(a, b, [-1, -2, -3]);
eig(a-b*k);

f = place(a', c', [-1, -2, -3]);
f = f'
eig(a-f*c);

xp = ss(a-f*c, -f, -k, 0);
np = ss(a-b*k, b, c - d*k, d);
yp = ss(a-f*c, -b + f*d, -k, 1);
dp = ss(a-b*k, b, -k, 1);

[xp_tf_n, xp_tf_d] = ss2tf(xp.A, xp.B, xp.C, xp.D, 1);
[np_tf_n, np_tf_d] = ss2tf(np.A, np.B, np.C, np.D, 1);
[yp_tf_n, yp_tf_d] = ss2tf(yp.A, yp.B, yp.C, yp.D, 1);
[dp_tf_n, dp_tf_d] = ss2tf(dp.A, dp.B, dp.C, dp.D, 1);

xp_tf = tf(xp_tf_n, xp_tf_d);
np_tf = tf(np_tf_n, np_tf_d);
yp_tf = tf(yp_tf_n, yp_tf_d);
dp_tf = tf(dp_tf_n, dp_tf_d);

simplify(xp_tf*np_tf + yp_tf*dp_tf)

r = 10;
C = (xp_tf + r*dp_tf)/(yp_tf - r*np_tf);

step(feedback(P*C, 1))