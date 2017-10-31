clear;clc;


% xmax=0.5;
% ymax=1;
% tmax=10;
% t=1/tmax*linspace(0,tmax,101)';
% x=[zeros(25,1); linspace(0,xmax,25)'; xmax*ones(25,1); linspace(xmax,0,25)'; 0];
% y=[linspace(0,ymax,25)'; ymax*ones(25,1); linspace(ymax,0,25)'; zeros(25,1); 0];
% f=@(x) tanh(x);%sigmoid or tanh or whatever
% fprime=@(x) (1-tanh(x).*tanh(x)) ;%deriv
% 
% txy=[t x y];
% txydown=downsample(txy,5);
% 
% %weighting matrix
% nmat=2*rand(101,21)-1;
% 
% alpha=0.01;
% windFromWest=[zeros(21,1) 0.01*ones(21,1) zeros(21,1)];
% for i=1:10000
% 	L1=f(nmat'*txy);
%     L1operated=L1+windFromWest;  %mess with L1
%     err=txydown-L1operated;
%     d_L1=err.*fprime(L1operated);
% 	nmat=nmat+alpha*txy*d_L1';
% 	%[ttraj, xtraj, ytraj]=callMS(reweight(:,1),reweight(:,2),reweight(:,3));
% 	
% end
% %f(nmat)'*txy
% errAfterLearning=L1operated-txydown

% %t=0:.05:2*pi;
% %x=sin(t);
% %xdown=downsample(x,3);
% X=[0 0 1;1 1 1; 1 0 1; 0 1 1];
% Y=[0;1;1;0];
% f=@(x) 1./(1+exp(-x));
% fprime=@(x) x.*(1-x);
% syn0=2*rand(3,4)-1;
% syn1=2*rand(4,1)-1;
% a0=0.1; a1=0.1;
% for ijk=1:1000
%     L1=f(X)*syn0;
%     L2=f(L1)*syn1;
%     dL2=(Y-L2).*fprime(L2);
%     dL1=syn1*dL2'*fprime(L1);
%     size(dL1)
%     syn1=syn1+a1*L1'*dL2;
%     syn0=syn0+a0*X'*dL1;
% end
% L2

% X=[[0;0;0.5;1] [0;1;0;1] [2;2;2;2] ];
% Y=[0;0;0.5;1];
% f=@(x) 1./(1+exp(-x));
% fprime=@(x) 2*x.*(1-x);
% syn0=2*rand(3,1)-1;
% alpha=2;
% for x=1:10000
%     L1=f(X*syn0);
%     err1=Y-L1;
%     dL1=err1.*fprime(L1);
%     syn0=syn0+alpha*X'*dL1
% end
% L1



