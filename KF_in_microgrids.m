A = [175.9, 176.8, 511, 103.6; -350, 0, 0, 0; -544.2, -474.8, -408.8, -828.8; -119.7, -554.6, -968.8, -1077.5];
B = [0.8, 334.2, 525.1, -103.6; -350, 0, 0, 0; -69.3, -66.1, -420.1, -828.8; -434.9, -414.2, -108.7, -1077.5];
%C = ? Randomly taken here
C = [1, 1, 1, 1];
D = 0;

step_size = 0.00001;
I = eye(4,4);

A_d = I + A*step_size;
B_d = B*step_size;

n = randn(4,1);
w = rand(4,1);

cov_n = cov(n);
cov_w = cov(w);

sys = ss(A_d, B_d, C, D, -1);

[KF, L, P, M] = kalman(sys, cov_n, cov_w);

KF = KF(1,:);

disp(M); %innovation gain 

%Building the plant model and putting it parallel to the estimated model
a = A;
b = [B B 0*B 0*B];
c = [C ;C];
d = [0 0 0 0; 0 0 0 0]



