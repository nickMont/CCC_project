clear;clc;


%run trajectory on a range from 0.1 to 0.9

xmax=0.4;
ymax=0.8;
tmax=10;
t=1/tmax*linspace(0,tmax,101)';
x=0.1+[zeros(25,1); linspace(0,xmax,25)'; xmax*ones(25,1); linspace(xmax,0,25)'; 0];
y=0.1+[linspace(0,ymax,25)'; ymax*ones(25,1); linspace(ymax,0,25)'; zeros(25,1); 0];
f=@(x) tanh(x);%sigmoid or tanh or whatever
fprime=@(x) (1-tanh(x).*tanh(x)) ;%deriv

txy=[t x y];
txydown=downsample(txy,5);

%weighting matrix
nmat=2*rand(101,21)-1;

alpha=0.01;
windFromWest=[zeros(21,1) 0.01*ones(21,1) zeros(21,1)];
windFromWest(5:15,2)=0.02;
windFromWest(9:11,2)=0.04;
for i=1:1000
	L1=f(nmat'*txy);
    L1operated=L1+windFromWest;  %mess with L1
    err=txydown-L1operated;
    d_L1=err.*fprime(L1operated);
	nmat=nmat+alpha*txy*d_L1';
	%[ttraj, xtraj, ytraj]=callMS(reweight(:,1),reweight(:,2),reweight(:,3));
	
end
%f(nmat)'*txy
errAfterLearning=L1operated-txydown



