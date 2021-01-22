clear;
clc;

A = [175.9, 176.8, 511, 103.6; -350, 0, 0, 0; -544.2, -474.8, -408.8, -828.8; -119.7, -554.6, -968.8, -1077.5];
B = [0.8, 334.2, 525.1, -103.6; -350, 0, 0, 0; -69.3, -66.1, -420.1, -828.8; -434.9, -414.2, -108.7, -1077.5];
C = [1, 0, 1, 0];
D = 0;

step_size = 0.00001;
I = eye(4,4);

A_d = I + A*step_size;
B_d = B*step_size;

Plant = ss(A_d,B_d,C,0,-1,'inputname',{'u' 'v' 'w' 'x' },'outputname','y');

Q = 2.3; %noise covariances
R = 1;

[kalmf,L,~,M,Z] = kalman(Plant,Q,R);


% kalmf = kalmf(1,:);

%disp(M);

a = A;
b = [0.8, 334.2, 525.1, -103.6, 0; -350, 0, 0, 0, 0; -69.3, -66.1, -420.1, -828.8, 0; -434.9, -414.2, -108.7, -1077.5, 0]
c = [C ;C];
d = [0, 0, 0, 0, 0; 0, 0, 0, 0, 1];

P = ss(a,b,c,d,-1,'inputname',{'u' 'v' 'w' 'x' 'z'},'outputname',{'y' 'yv'});

sys = parallel(P,kalmf,1,1,[],[])

%sys(1,4)
SimModel = feedback(sys,1,8,2,1);   %for MIMO system mention the inputs and outputs
SimModel = SimModel([1 3 4 5 6],[1 2 3 4 5])   % Delete yv form I/O

SimModel.inputname
SimModel.outputname

t = (0:100)';


rng(1,'twister');

u = randn(length(t),1);
w = randn(length(t),1);
v = randn(length(t),1);
x = randn(length(t),1);
z = randn(length(t),1);

out = lsim(SimModel,[v,w,x,z,u])

y = out(:,1);   % true response
ye = out(:,2);  % filtered response
yv = y + v;     % measured response

clf
subplot(111), plot(t,y,'g',t,ye,'b'),
xlabel('No. of samples'), ylabel('Output')
title('Kalman filter response')


