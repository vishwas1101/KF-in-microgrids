% paper implementation of Kalman Filter Based Microgrid State Estimation
% and Control Using the IoT with 5G Networks
clear;
clc;

%%%%%%%%%%%%%%%Output Estimate Starts Here%%%%%%%%%%%%%%%%%

A = [1.03, -.01, .01; .001, 1.02, -.01; .001, -.1, 1.05];
B = [0.6, 0.1, 0.2; 0.1 0.7 0.15; 0.2, 0.15, 0.8];
C = [1, 1, 1];
D = 0;

Plant = ss(A, B, C, 0, -1, 'inputname',{'u', 'w', 'x'}, 'outputname', 'y');

Q = 1000; % covariance process noise
R = 1000; % covariance observation noise

[KF, L, p, M, Z] = kalman(Plant, Q, R);

a = A;
b = [0.6, 0.1, 0.2, 0; 0.1, 0.7, 0.15, 0; 0.2, 0.15, 0.8, 0];
c = [1, 1, 1; 1, 1, 1];
d = [0, 0, 0, 0; 0, 0, 0, 1];

P = ss(a, b, c, d, -1, 'inputname', {'u', 'w', 'x', 'v'}, 'outputname', {'y', 'yv'});

sys = parallel(P, KF, 1, 1, [], []);

model = feedback(sys, 1, 6, 2, 1);  %for MIMO system mention the inputs and outputs
model = model([1, 3],[2, 3, 4, 5]); % Delete yv form I/O

model.inputname;
model.outputname;

t = (0:100)';

rng(1, 'twister');
u = randn(length(t), 1);
w = randn(length(t), 1);
v = randn(length(t), 1);
x = randn(length(t), 1);

output = lsim(model,[w, v, u, x]);

y_true = output(:, 1);   % true response
y_estimate = output(:, 2);  % estimated response
y_measured = y_true + v; %measured response 

y_estimate(1,1) = y_true(1,1);

MSR_KF = (0.0083*(sum(abs(y_estimate - y_true))^2)^0.5)
MSR_measure = (0.0083*(sum(abs(y_measured - y_true))^2)^0.5)

%%%%%%%%%%%%%%%State Estimate 1 Starts Here%%%%%%%%%%%%%%%%%

C1 = [1, 0, 0];

Plant1 = ss(A, B, C1, 0, -1, 'inputname',{'u', 'w', 'x'}, 'outputname', 'y');

[KF1, L1, p1, M1, Z1] = kalman(Plant1, Q, R);

c1 = [1, 0, 0; 1, 0, 0];

P1 = ss(a, b, c1, d, -1, 'inputname', {'u', 'w', 'x', 'v'}, 'outputname', {'y', 'yv'});

x1_initial = pinv([C1*(C1)'])*(C1)'; 

sys1 = parallel(P1, KF1, 1, 1, [], []);

modelEstimate1 = feedback(sys1, 1, 6, 2, 1);
modelEstimate1 = modelEstimate1([1, 3],[2, 3, 4, 5]);

modelEstimate1.inputname;
modelEstimate1.outputname;

state1 = lsim(modelEstimate1,[w, v, u, x]);

x1_true = state1(:, 1);   
x1_estimate = state1(:, 2);  
x1_measured = x1_true + v; 

x1_estimate(1,1) = x1_true(1,1);

%%%%%%%%%%%%%%%State Estimate 2 Starts Here%%%%%%%%%%%%%%%%%

C2 = [0, 1, 0];

Plant2 = ss(A, B, C2, 0, -1, 'inputname',{'u', 'w', 'x'}, 'outputname', 'y');

[KF2, L2, p2, M2, Z2] = kalman(Plant2, Q, R);

c2 = [0, 1, 0; 0, 1, 0];

P2 = ss(a, b, c2, d, -1, 'inputname', {'u', 'w', 'x', 'v'}, 'outputname', {'y', 'yv'});

x2_initial = pinv([C2*(C2)'])*(C2)'; 

sys2 = parallel(P2, KF2, 1, 1, [], []);

modelEstimate2 = feedback(sys2, 1, 6, 2, 1);
modelEstimate2 = modelEstimate2([1, 3],[2, 3, 4, 5]);

modelEstimate2.inputname;
modelEstimate2.outputname;

state2 = lsim(modelEstimate2,[w, v, u, x]);

x2_true = state2(:, 1);   
x2_estimate = state2(:, 2);  
x2_measured = x2_true + v; 

x2_estimate(1,1) = x2_true(1,1);

%%%%%%%%%%%%%%%State Estimate 3 Starts Here%%%%%%%%%%%%%%%%%

C3 = [0, 0, 1];

Plant3 = ss(A, B, C3, 0, -1, 'inputname',{'u', 'w', 'x'}, 'outputname', 'y');

[KF3, L3, p3, M3, Z3] = kalman(Plant3, Q, R);

c3 = [0, 0, 1; 0, 0, 1];

P3 = ss(a, b, c3, d, -1, 'inputname', {'u', 'w', 'x', 'v'}, 'outputname', {'y', 'yv'});

x3_initial = pinv([C3*(C3)'])*(C3)';

sys3 = parallel(P3, KF3, 1, 1, [], []);

modelEstimate3 = feedback(sys3, 1, 6, 2, 1);
modelEstimate3 = modelEstimate3([1, 3],[2, 3, 4, 5]);

modelEstimate3.inputname;
modelEstimate3.outputname;

state3 = lsim(modelEstimate3,[w, v, u, x]);

x3_true = state3(:, 1);   
x3_estimate = state3(:, 2);  
x3_measured = x3_true + v; 

x3_estimate(1, 1) = x3_true(1, 1);

%%%%%%%%%%%%%%%%%%Plotting Results%%%%%%%%%%%%%%%%%

clf
subplot(411);
plot(t, y_true, 'g', t, y_estimate, 'b');
xlabel('samples'), ylabel('output')
title('Output Estimate 1')

subplot(412);
plot(t, x1_true, 'g', t, x1_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 1')

subplot(413);
plot(t, x2_true, 'g', t, x2_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 2')

subplot(414);
plot(t, x3_true, 'g', t, x3_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 3')

%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%
