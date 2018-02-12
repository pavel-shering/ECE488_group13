B_prime = [1 1 1]'

A_prime = [-2    -1     1;
            0    -1     0;
           -2     0     0]

%% Plug a new A_prime and B_prime
% A_prime = []
% B_prime = []
       
p = [-2.0000001 -2.0000005 -2.00004];
k = place(A_prime, B_prime, p)

eig(A_prime -  B_prime*k)

