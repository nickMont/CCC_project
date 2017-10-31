clear;clc;


%run trajectory on a range from 0.1 to 0.9

xmax=0.2;
ymax=0.4;
tmax=10;
t_in=1/tmax*linspace(0,tmax,101)';
dt_default=min(diff(t_in));
%x_in=0.1+[zeros(25,1); linspace(0,xmax,25)'; xmax*ones(25,1); linspace(xmax,0,25)'; 0];
%y_in=0.1+[linspace(0,ymax,25)'; ymax*ones(25,1); linspace(ymax,0,25)'; zeros(25,1); 0];
x_in=0.5+xmax*t_in.*cos(2*pi*t_in);
y_in=0.5+ymax*sin(2*pi*t_in);
f=@(x) tanh(x);%sigmoid or tanh or whatever
fprime=@(x) (1-tanh(x).*tanh(x)) ;%deriv

txy=[x_in y_in];
txydown=downsample(txy,5);
dt_down=max(t_in)/(length(txydown)-1);
t_samp=(0:dt_down:max(t_in))';

%weighting matrix
mismSize=61;
n1start=0.5; n2start=0.05;
nmat1=n1start*rand(101,mismSize)-n1start/2;
nmat2=n2start*rand(mismSize,21)-n2start/2;
% nmat=nmat';
% nmat(1,:)=[1 zeros(1,100)];
% noff=0;
% for ijk=2:20
%     nmat(ijk,:)=[zeros(1,5*(ijk-1)+noff) 1 zeros(1,100-5*(ijk-1)-noff)];
% end
% nmat(21,:)=[zeros(1,100) 1];
% nmat=nmat';

a1=0.01;
a2=0.01;

figure(1);clf;
plot(txy(:,1),txy(:,2));

for i=1:1000
	%L1=f(nmat'*txy);
    %L1=nmat'*txy;
    L1=f(nmat1')*txy;
    L2=f(nmat2')*L1;
    [solCoeff,J]=callMinSnap2D([t_samp L2]);
    
%     if min(diff(L1(:,1)))<0
%         i
%         fprintf('time error')
%         break;  %if time ordering fails, then this example is done
%     end
    [xop,yop]=processSolCoeff(t_samp,solCoeff,dt_down);
%    [xop,yop,~]=processSolCoeff(L1(:,1), solCoeff, max(L1(:,1))/20);
    L2operated=[xop' yop'];  %mess with L1

    err=txydown-L2operated;
    dL2=err.*fprime(L2operated);
    dL1=nmat2*dL2.*fprime(L1);
    nmat2=nmat2+a2*L1*dL2';
	nmat1=nmat1+a1*txy*dL1';
    errAfterLearning=err
    
    if max(max(abs(errAfterLearning)))<1e-8
        i
        fprintf('state convergence')
        break;  %if sufficiently accurate; overtraining is a problem
    end
	
end
%f(nmat)'*txy
%errAfterLearning=err
figure(1);
hold on;
plot(L2operated(:,1),L2operated(:,2),'r')
axis([0 1 0 1]);
legend('Before','After');




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





