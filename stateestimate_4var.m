clear;
clc;

A = [175.9, 176.8, 511, 103.6; -350, 0, 0, 0; -544.2, -474.8, -408.8, -828.8; -119.7, -554.6, -968.8, -1077.5];
B = [0.8, 334.2, 525.1, -103.6; -350, 0, 0, 0; -69.3, -66.1, -420.1, -828.8; -434.9, -414.2, -108.7, -1077.5];
C = [1, 0, 0, 0];
D = 0;

step_size = .00001;
I = eye(4,4);

A_d = I + A*step_size;
B_d = B*step_size;

n = randn(4,1);
o = randn(4,1);

cov_n = cov(n);
cov_w = cov(o);

plant = ss(A_d, B_d, C, D, -1, 'inputname', {'u' 'v' 'w' 'x'}, 'outputname', 'y');

[KF, L, P, M, Z] = kalman(plant, cov_n, cov_w);

%the 2,3,4,5 groups are the state estimates
%so we keep only that 
% KF = KF([2 3 4 5],:); 

%disp(M); %innovation gain 

%Building the plant model and putting it parallel to the estimated model
a = A;
b = [0.8, 334.2, 525.1, -103.6, 0; -350, 0, 0, 0, 0; -69.3, -66.1, -420.1, -828.8, 0; -434.9, -414.2, -108.7, -1077.5, 0];
c = [C ;C];
d = [0, 0, 0, 0, 0; 0, 0, 0, 0, 1];

Plant1 = ss(a, b, c, d, -1, 'inputname', {'u' 'v' 'w' 'x' 'z'}, 'outputname', {'y' 'yv'});

% no we put it in parallel with the estimated block
sys = parallel(Plant1, KF, 1, 1, [], [])

simModel = feedback(sys, 1, 8, 2, 1);
simModel = simModel([1 3 4 5 6 7], [3 4 5 6 7]);

simModel.inputname;
simModel.outputname;

% now plot the results 
t = (0:100)';

rng(10,'twister');

u = randn(length(t),1);
w = randn(length(t),1);
v = randn(length(t),1);
x = randn(length(t),1);
z = randn(length(t),1);

out = lsim(simModel,[x, z, u, v, w]);

y = out(:,1);   % true response
ye = out(:,2);  % filtered response

clf
subplot(111), plot(t,y,'g',t,ye,'b'),
xlabel('No. of samples'), ylabel('Output')
title('Kalman filter response')
