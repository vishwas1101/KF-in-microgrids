clear;
clc;

%%%%%%%%%%%%%%%Output Estimate Starts Here%%%%%%%%%%%%%%%%%

% matrices as per the matlab example
A = [1.1269, -0.4940, 0.1129; 1.0000, 0, 0; 0, 1.0000, 0];
B = [-0.3832; 0.5919; 0.5191];
C = [1, 1, 1];
D = 0;

outputEstimatePlant = ss(A,[B B], C, 0, -1, 'inputname',{'u', 'w'}, 'outputname', 'y');

Q = .5; % covariance process noise
R = .3; % covariance observation noise

% applying the filter to the above state space model
[outputEstimateKF, L, p, M, Z] = kalman(outputEstimatePlant, Q, R); % the filter

% another plant to put it in parallel with the filter using v as the
% dummy input
a = A;
b = [B, B, 0*B];
c = [C; C];
d = [0, 0, 0; 0, 0, 1];

OutputEstimateP = ss(a, b, c, d, -1, 'inputname', {'u', 'w', 'v'}, 'outputname', {'y', 'yv'});

% putting both in parallel and using the first input as the shared input. 
sys = parallel(OutputEstimateP, outputEstimateKF, 1, 1, [], []);

% connecting the filter input to the plant output.  
modeloutputEstimate = feedback(sys, 1, 4, 2, 1); 
% taking the estimate and true value, along with the inputs
modeloutputEstimate = modeloutputEstimate([1, 3],[1, 2, 3]); 

% random inputs and 100 samples 
t = (0:100)';
rng(1, 'twister');

u = randn(length(t), 1);
w = randn(length(t), 1);
v = randn(length(t), 1);

outputEstimate = lsim(modeloutputEstimate,[w, v, u]);

y_true = outputEstimate(:, 1);   
y_estimate = outputEstimate(:, 2);  
y_measured = y_true + v; 

y_estimate(1,1) = y_true(1,1);

% MSR_KF = (0.0083*(sum(abs(y_estimate - y_true))^2)^0.5);
% MSR_measure = (0.0083*(sum(abs(y_measured - y_true))^2)^0.5);

%%%%%%%%%%%%%%%State Estimate 2 Starts Here%%%%%%%%%%%%%%%%%

% changed the output matrix c here to get my output as y = x1
C2 = [0, 1, 0];

% similar steps are followed for the rest of the estimates

stateEstimatePlant2 = ss(A,[B B], C2, 0, -1, 'inputname',{'u', 'w'}, 'outputname', 'y');

[stateEstimateKF2, L2, p2, M2, Z2] = kalman(stateEstimatePlant2, Q, R);

c2 = [C2; C2]; 

stateEstimateP2 = ss(a, b, c2, d, -1, 'inputname', {'u', 'w', 'v'}, 'outputname', {'y', 'yv'});
sys2 = parallel(stateEstimateP2, stateEstimateKF2, 1, 1, [], []);

modelStateEstimate2 = feedback(sys2, 1, 4, 2, 1);
modelStateEstimate2 = modelStateEstimate2([1, 3], [1, 2, 3]); 

stateEstimate2 = lsim(modelStateEstimate2,[w, v, u]);

x2_true = stateEstimate2(:, 1);   
x2_estimate = stateEstimate2(:, 2);  
x2_measured = x2_true + v; 

% initial values
x2_initial = pinv([C2*(C2)'])*(C2)' * y_true(1);
x2_estimate(1,1) = x2_initial(2,1);

%%%%%%%%%%%%%%%State Estimate 1 Starts Here%%%%%%%%%%%%%%%%%

C1 = [1, 0, 0];
% C1 = [1, 1, 0];

stateEstimatePlant1 = ss(A,[B B], C1, 0, -1, 'inputname',{'u', 'w'}, 'outputname', 'y');

[stateEstimateKF1, L1, p1, M1, Z1] = kalman(stateEstimatePlant1, Q, R);

c1 = [C1; C1]; 

stateEstimateP1 = ss(a, b, c1, d, -1, 'inputname', {'u', 'w', 'v'}, 'outputname', {'y', 'yv'});
sys1 = parallel(stateEstimateP1, stateEstimateKF1, 1, 1, [], []);

modelStateEstimate1 = feedback(sys1, 1, 4, 2, 1);
modelStateEstimate1 = modelStateEstimate1([1, 3], [1, 2, 3]); 

stateEstimate1 = lsim(modelStateEstimate1,[w, v, u]);

x1_true = stateEstimate1(:, 1);   
x1_estimate = stateEstimate1(:, 2);  
x1_measured = x1_true + v; 

% x1_true = stateEstimate1(:, 1) - stateEstimate2(:, 1);   
% x1_estimate = stateEstimate1(:, 2) - stateEstimate2(:, 2);  
% x1_measured = x1_true + v; 

x1_estimate(1,1) = x1_true(1,1);

MSR_KF1 = (0.0083*(sum(abs(x1_estimate - x1_true))^2)^0.5);
MSR_measure1 = (0.0083*(sum(abs(x1_measured - x1_true))^2)^0.5);

x1_initial = pinv([C1*(C1)'])*(C1)' * y_true(1);
x1_estimate(1,1) = x1_initial(1,1);

%%%%%%%%%%%%%%%State Estimate 3 Starts Here%%%%%%%%%%%%%%%%%

C3 = [0, 0, 1];

stateEstimatePlant3 = ss(A,[B B], C3, 0, -1, 'inputname',{'u', 'w'}, 'outputname', 'y');

[stateEstimateKF3, L3, p3, M3, Z3] = kalman(stateEstimatePlant3, Q, R);

c3 = [C3; C3]; 

stateEstimateP3 = ss(a, b, c3, d, -1, 'inputname', {'u', 'w', 'v'}, 'outputname', {'y', 'yv'});
sys3 = parallel(stateEstimateP3, stateEstimateKF3, 1, 1, [], []);

modelStateEstimate3 = feedback(sys3, 1, 4, 2, 1);
modelStateEstimate3 = modelStateEstimate3([1, 3], [1, 2, 3]); 

stateEstimate3 = lsim(modelStateEstimate3,[w, v, u]);

x3_true = stateEstimate3(:, 1);   
x3_estimate = stateEstimate3(:, 2);  
x3_measured = x3_true + v; 

x3_initial = pinv([C1*(C1)'])*(C1)' * y_true(1);
x3_estimate(1,1) = x3_initial(3,1);

y_estimate(1,1) = C*[x1_initial(1,1); x2_initial(2,1); x3_initial(3,1)];

%%%%%%%%%%%%%%%%%%Plotting Results%%%%%%%%%%%%%%%%%

clf
subplot(411);
plot(t, y_true, 'g', t, y_estimate, 'b');
xlabel('samples'), ylabel('output')
title('Output Estimate')

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
