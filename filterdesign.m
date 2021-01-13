A = [1.1269, -0.4940, 0.1129; 1.0000, 0, 0; 0, 1.0000, 0];

B = [-0.3832
      0.5919
      0.5191];

C = [1 0 0];

D = 0;

Plant = ss(A,[B B],C,0,-1,'inputname',{'u' 'w'},'outputname','y')

Q = 2.3; %noise covariances
R = 1;

[kalmf,L,~,M,Z] = kalman(Plant,Q,R);

kalmf = kalmf(1,:);

disp(M);

a = A;
b = [B B 0*B];
c = [C;C];
d = [0 0 0;0 0 1];
P = ss(a,b,c,d,-1,'inputname',{'u' 'w' 'v'},'outputname',{'y' 'yv'})

sys = parallel(P,kalmf,1,1,[],[]);

SimModel = feedback(sys,1,4,2,1);
SimModel = SimModel([1 3],[1 2 3]);     % Delete yv form I/O

SimModel.inputname
SimModel.outputname

t = (0:100)';
u = -t/10;

rng(10,'twister');
w = sqrt(Q)*randn(length(t),1);
v = sqrt(R)*randn(length(t),1);

out = lsim(SimModel,[w,v,u]);

y = out(:,1);   % true response
ye = out(:,2);  % filtered response
yv = y + v;     % measured response

clf
subplot(111), plot(t,y,'b',t,ye,'r--'),
xlabel('No. of samples'), ylabel('Output')
title('Kalman filter response')


