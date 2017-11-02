clear;clc;clf;


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

txy=[t_in x_in y_in];
txydown=downsample(txy,5);
dt_down=min(diff(txydown(:,1)));
t_samp=(0:dt_down:max(t_in))';

%weighting matrix
nmat=2*rand(101,21)-1;
nmat=nmat';
nmat(1,:)=[1 0*ones(1,100)];
noff=1;
for ijk=2:20
    nmat(ijk,:)=[zeros(1,5*(ijk-1)+noff) 1 zeros(1,100-5*(ijk-1)-noff)];
end
nmat(21,:)=[zeros(1,100) 1];
nmat=nmat';

alpha=0.01;
% windFromWest=[zeros(21,1) 0.01*ones(21,1) zeros(21,1)];
% windFromWest(5:15,2)=0.02;
% windFromWest(9:11,2)=0.04;
for i=1:1000
	%L1=f(nmat'*txy);
    L1=nmat'*txy;
    %L1=f(nmat')*txy;
    L1sort = sortrows(L1,1);
    [solCoeff,J]=callMinSnap2D(L1sort,t_in);
    
%     if min(diff(L1(:,1)))<0
%         i
%         fprintf('time error')
%         break;  %if time ordering fails, then this example is done
%     end
    [xop,yop]=processSolCoeff(L1(:,1),solCoeff,dt_down);
%    [xop,yop,~]=processSolCoeff(L1(:,1), solCoeff, max(L1(:,1))/20);
    L1operated=[L1(:,1) xop' yop'];  %mess with L1
    if i==1
        plot(L1operated(:,2),L1operated(:,3));
    end
    
    err=txydown-L1operated;
    d_L1=err.*fprime(L1operated);
	nmat=nmat+alpha*txy*d_L1';
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
plot(L1operated(:,2),L1operated(:,3),'r')
axis([0 1 0 1]);
legend('Before','After');

nmatBottomRightHandCorner=nmat(87:101,18:21)




