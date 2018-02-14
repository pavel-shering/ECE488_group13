B_prime = [1 1 1]'

A_prime = [-2    -1     1;
            0    -1     0;
           -2     0     0]

%% Plug a new A_prime and B_prime
% A_prime = []
% B_prime = []
A_prime = [-1.0000         0         0;
           -4.5000   -2.0000    2.5000;
           -5.0000   -2.0000    2.0000];

B_prime = [ 1 2 3]';

p = [-1.0000001 -1.0000005 -1.00004];
k = place(A_prime, B_prime, p)

eig(A_prime -  B_prime*k)

