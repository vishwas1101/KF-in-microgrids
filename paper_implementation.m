% paper implementation of Kalman Filter Based Microgrid State Estimation
% and Control Using the IoT with 5G Networks
clear;
clc;

A = [1.03, 0, 0; 0, 1.02, 0; 0, 0, 1.05];
B = [0.6, 0.1, 0.2; 0.1 0.7 0.15; 0.2, 0.15, 0.8];
C = [1, 1, 1; 1 0 0; 0 1 0; 0 0 1];
D = 0;

Plant = ss(A, B, C, 0, -1, 'inputname',{'u', 'w', 'x'}, 'outputname', {'y', 'y1', 'y2', 'y3'});

Q = 0.001; % covariance process noise
R = 0.001; % covariance observation noise

[KF, L, p, M, Z] = kalman(Plant, Q, R);

a = A;
b = [0.6, 0.1, 0.2, 0; 0.1, 0.7, 0.15, 0; 0.2, 0.15, 0.8, 0];
c = [1, 1, 1; 1, 1, 1; 1 0 0; 0 1 0; 0 0 1];
d = [0, 0, 0, 0; 0, 0, 0, 1; 0 0 0 0; 0 0 0 0; 0 0 0 0];

P = ss(a, b, c, d, -1, 'inputname', {'u', 'w', 'x', 'v'}, 'outputname', {'y', 'yv', 'y1', 'y2', 'y3'});

sys = parallel(P, KF, 1, 1, [], [])

model = feedback(sys, 1, 6, 2, 1);   %for MIMO system mention the inputs and outputs
model = model([1, 3, 4, 5, 6, 7, 8, 9],[2, 3, 4, 5]); % Delete yv form I/O

model.inputname
model.outputname

t = (0:50)';

rng(1, 'twister');

u = randn(length(t), 1);
w = randn(length(t), 1);
v = randn(length(t), 1);
x = randn(length(t), 1);

output = lsim(model,[w, v, u, x])
% output2 = lsim(P,[w, v, u]);

y_true = output(:, 1);   % true response
y_estimate = output(:, 5);  % estimated response
y_measured = y_true + v; %measured response 
x1 = output(:, 2); % state estimate 1
x1_e = output(:, 6);  
x2 = output(:, 3); % state estimate 2
x2_e = output(:, 7); 
x3 = output(:, 4); % state estimate 3
x3_e = output(:, 8);

MSR_KF = (0.0083*(sum(abs(y_estimate - y_true))^2)^0.5)
MSR_measure = (0.0083*(sum(abs(y_measured - y_true))^2)^0.5)

clf

subplot(411);
plot(t, x1, 'g', t, x1_e, 'b');
xlabel('samples'), ylabel('output')
title('state estimate 1')

subplot(412);
plot(t, x2, 'g', t, x2_e, 'b');
xlabel('samples'), ylabel('output')
title('state estimate 2')

subplot(413);
plot(t, x3, 'g', t, x3_e, 'b');
xlabel('samples'), ylabel('output')
title('state estimate 3')

subplot(414);
plot(t, y_true, 'g', t, y_estimate, 'b');
xlabel('samples'), ylabel('output')
title('Kalman filter')


