% Kalman Filter Based Microgrid State Estimation Using the Internet of Things Communication Network

clear;
clc;

%%%%%%% Output Estimate %%%%%%%
A = [175.9, 176.8, 511, 103.6;-350, 0, 0, 0;-544.2, -474.8, -408.8, -828.8; -119.7, -554.6, -968.8, -1077.5];
B = [0.8, 334.2, 525.1, -103.6;-350, 0, 0, 0;-69.3, -66.1, -420.1, -828.8; -434.9, -414.2, -108.7, -1077.5];
C = [1, 1, 1, 1];

step_size = 0.0001;
I = eye(4,4);

A_d = I + A*step_size;
B_d = B*step_size;

Plant = ss(A_d, B_d, C, 0, -1, 'inputname',{'p', 'q', 'r', 's'}, 'outputname', 'y');
eig(Plant)
Q = 2.3; 
R = 1.2;

[KF, L, ~, M, Z] = kalman(Plant, Q, R);

a_d = A_d;
b = [0.8, 334.2, 525.1, -103.6, 0; -350, 0, 0, 0, 0; -69.3, -66.1, -420.1, -828.8, 0; -434.9, -414.2, -108.7, -1077.5, 0];
c = [C; C];
d = [0, 0, 0, 0, 0; 0, 0, 0, 0, 1];

b_d = b*step_size;

P = ss(a_d, b_d, c, d, -1, 'inputname',{'p', 'q', 'r', 's', 'u'}, 'outputname', {'y', 'yv'});

sys = parallel(P, KF, [1, 2, 3], [1, 2, 3], [], []);

model = feedback(sys, 1, 6, 2, 1);

model = model([1, 3],[1, 2, 3, 4, 5]);

model.inputname;

t = (0:100)';
rng(1, 'twister');

p = t/8;
q = randn(length(t), 1);
r = randn(length(t), 1);
s = randn(length(t), 1);
u = randn(length(t), 1);

output = lsim(model,[q, r, s, u, p]);

y_true = output(:, 1);
y_estimate = output(:, 2);
y_measured = y_true + t;

subplot(511);
plot(t, y_true, 'g', t, y_estimate, 'b');
xlabel('samples'), ylabel('output')
title('Output Estimate')

%%%%% State Estimate 1 %%%%%%%

C1 = [1, 0, 0, 0];

Plant1 = ss(A_d, B_d, C1, 0, -1, 'inputname',{'p', 'q', 'r', 's'}, 'outputname', 'y');

[KF1, L, ~, M, Z] = kalman(Plant1, Q, R);

c1 = [C1; C1];

P1 = ss(a_d, b_d, c1, d, -1, 'inputname',{'p', 'q', 'r', 's', 'u'}, 'outputname', {'y', 'yv'});

sys1 = parallel(P1, KF1, [1, 2, 3], [1, 2, 3], [], []);

model1 = feedback(sys1, 1, 6, 2, 1);

model1 = model1([1, 3],[1, 2, 3, 4, 5]);

output1 = lsim(model1,[q, r, s, u, p]);

x1_true = output1(:, 1);
x1_estimate = output1(:, 2);
x1_measured = x1_true + t;

subplot(512);
plot(t, x1_true, 'g', t, x1_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 1')

%%%%% State Estimate 2 %%%%%%%

C2 = [0, 1, 0, 0];

Plant2 = ss(A_d, B_d, C2, 0, -1, 'inputname',{'p', 'q', 'r', 's'}, 'outputname', 'y');

[KF2, L, ~, M, Z] = kalman(Plant2, Q, R);

c2 = [C2; C2];

P2 = ss(a_d, b_d, c2, d, -1, 'inputname',{'p', 'q', 'r', 's', 'u'}, 'outputname', {'y', 'yv'});

sys2 = parallel(P2, KF2, [1, 2, 3], [1, 2, 3], [], []);

model2 = feedback(sys2, 1, 6, 2, 1);

model2 = model2([1, 3],[1, 2, 3, 4, 5]);

output2 = lsim(model2,[q, r, s, u, p]);

x2_true = output2(:, 1);
x2_estimate = output2(:, 2);
x2_measured = x2_true + t;

subplot(513);
plot(t, x2_true, 'g', t, x2_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 2')

%%%%% State Estimate 3 %%%%%%%

C3 = [0, 0, 1, 0];

Plant3 = ss(A_d, B_d, C3, 0, -1, 'inputname',{'p', 'q', 'r', 's'}, 'outputname', 'y');

[KF3, L, ~, M, Z] = kalman(Plant3, Q, R);

c3 = [C3; C3];

P3 = ss(a_d, b_d, c3, d, -1, 'inputname',{'p', 'q', 'r', 's', 'u'}, 'outputname', {'y', 'yv'});

sys3 = parallel(P3, KF3, [1, 2, 3], [1, 2, 3], [], []);

model3 = feedback(sys3, 1, 6, 2, 1);

model3 = model3([1, 3],[1, 2, 3, 4, 5]);

output3 = lsim(model3,[q, r, s, u, p]);

x3_true = output3(:, 1);
x3_estimate = output3(:, 2);
x3_measured = x3_true + t;

subplot(514);
plot(t, x3_true, 'g', t, x3_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 3')

%%%%% State Estimate 4 %%%%%%%

C4 = [0, 0, 0, 1];

Plant4 = ss(A_d, B_d, C4, 0, -1, 'inputname',{'p', 'q', 'r', 's'}, 'outputname', 'y');

[KF4, L, ~, M, Z] = kalman(Plant4, Q, R);

c4 = [C4; C4];

P4 = ss(a_d, b_d, c4, d, -1, 'inputname',{'p', 'q', 'r', 's', 'u'}, 'outputname', {'y', 'yv'});

sys4 = parallel(P4, KF4, [1, 2, 3], [1, 2, 3], [], []);

model4 = feedback(sys4, 1, 6, 2, 1);

model4 = model4([1, 3],[1, 2, 3, 4, 5]);

output4 = lsim(model4,[q, r, s, u, p]);

x4_true = output4(:, 1);
x4_estimate = output4(:, 2);
x4_measured = x4_true + t;

subplot(515);
plot(t, x4_true, 'g', t, x4_estimate, 'b');
xlabel('samples'), ylabel('output')
title('State Estimate 4')