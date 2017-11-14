clear;clc;
%Okay, I REALLY need to break this code up and make it more modular.

%run trajectory on a range from 0.1 to 0.9

rng(1)

xmax=0.2;
ymax=0.4;
tmax=10;
t_in=1/tmax*linspace(0,tmax,101)';
dt_default=min(diff(t_in));
f=@(x) tanh(x);%sigmoid or tanh or whatever
fprime=@(x) (1-tanh(x).*tanh(x)) ;%deriv
dt_down=0.05;
t_samp=(0:dt_down:max(t_in))';

%x_in=0.1+[zeros(25,1); linspace(0,xmax,25)'; xmax*ones(25,1); linspace(xmax,0,25)'; 0];
%y_in=0.1+[linspace(0,ymax,25)'; ymax*ones(25,1); linspace(ymax,0,25)'; zeros(25,1); 0];

%might want to just make the example array a separate file to avoid clutter
training_examples


%weighting matrix
mismSize=65;
n1start=0.6; n2start=0.02;
nmat1=n1start*rand(101,mismSize)-n1start/2;
nmat2=n2start*randn(mismSize,21)-n2start/2;
% nmat=nmat';
% nmat(1,:)=[1 zeros(1,100)];
% noff=0;
% for ijk=2:20
%     nmat(ijk,:)=[zeros(1,5*(ijk-1)+noff) 1 zeros(1,100-5*(ijk-1)-noff)];
% end
% nmat(21,:)=[zeros(1,100) 1];
% nmat=nmat';

a1=0.015;
a2=0.017;

% figure(1);clf;
% plot(txy(:,1),txy(:,2));
subint=1;
lenn=length(allSamples);

randkeep=zeros(mismSize,1);
randthresh=95/100;

maxsteps=1000;
fprintf('\nTraining network\n====================\n');
for i=1:maxsteps
    if(mod(i-1,maxsteps/20)==0)
        fprintf('=');
    end
    
    randkeep=ones(mismSize,1);
    for j=1:mismSize
        if(rand<randthresh)
            randkeep(j)=0;
        end
    end
    indkeep=find(randkeep==0);
    
    n=randsample(lenn,1);
    txy=allSamples{n};
    txydown=allDown{n};
    
    %L1=f(nmat'*txy);
    %L1=nmat'*txy;
    nmat1drop=nmat1(:,indkeep);
    nmat2drop=nmat2(indkeep,:);
    L1=f(nmat1drop')*txy;
    L2=f(nmat2drop')*L1;
    [solCoeff,~]=callMinSnap2D([t_samp L2],t_samp,subint);
    
    %     if min(diff(L1(:,1)))<0
    %         i
    %         fprintf('time error')
    %         break;  %if time ordering fails, then this example is done
    %     end
    [xop,yop]=processSolCoeff(t_samp,solCoeff,0.05);
    %    [xop,yop,~]=processSolCoeff(L1(:,1), solCoeff, max(L1(:,1))/20);
    L2operated=[xop' yop'];  %mess with L1
    
    err=txydown-L2operated;
    dL2=err.*fprime(L2operated);
    dL1=nmat2drop*dL2.*fprime(L1);
    nmat2(indkeep,:)=nmat2drop+a2*L1*dL2';
    nmat1(:,indkeep)=nmat1drop+a1*txy*dL1';
    %errAfterLearning=err;
    %maxabserr=max(max(abs(err)))
    
    %     if max(max(abs(errAfterLearning)))<1e-8
    %         i
    %         fprintf('state convergence')
    %         break;  %if sufficiently accurate; overtraining is a problem
    %     end
    
end
% %f(nmat)'*txy
% %errAfterLearning=err
% figure(1);
% hold on;
% plot(L2operated(:,1),L2operated(:,2),'r')
% axis([0 1 0 1]);
% legend('Target','with NN');


% n=1;
% txy=allSamples{n};
% txydown=allDown{n};
x_in=0.5+0.35*t_in.*cos(2.1*pi*t_in+pi/3);
y_in=0.5+0.25*sin(1.9*pi*t_in+0);
x_in=0.5+0.35*cos(2*pi*t_in);
y_in=0.5+0.35*sin(2*pi*t_in);
txy=[x_in y_in];
txydown=downsample(txy,5);

%L1=(1-randthresh)*f(nmat'*txy);
%L1=nmat'*txy;
L1=f(nmat1')*txy;
L2=randthresh*f(nmat2')*L1;
[solCoeff,J]=callMinSnap2D([t_samp L2],t_samp,subint);

[xop,yop]=processSolCoeff(t_samp,solCoeff,dt_down);
L2operated=[xop' yop'];  %mess with L1

err=txydown-L2operated;
dL2=err.*fprime(L2operated);
dL1=nmat2*dL2.*fprime(L1);
nmat2=nmat2+a2*L1*dL2';
nmat1=nmat1+a1*txy*dL1';
errAfterLearning=err
maxErrAfterLearning=max(max(abs(err)))



