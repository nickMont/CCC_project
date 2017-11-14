% %uncomment to run standalone
% rng(1)
% 
% tmax=10;
% t_in=1/tmax*linspace(0,tmax,101)';
% dt_default=min(diff(t_in));
% f=@(x) tanh(x);%sigmoid or tanh or whatever
% fprime=@(x) (1-tanh(x).*tanh(x)) ;%deriv
% dt_down=0.05;
% t_samp=(0:dt_down:max(t_in))';

nsamp=0;

%gen spirals
fxvec=[1.5*pi 2*pi 2.5*pi];
fyvec=fxvec;
xmaxvec=[0.1 0.2 0.3 0.4];
ymaxvec=xmaxvec;
dphixvec=[0 pi/4 pi/2];
dphiyvec=dphixvec;
for i1=1:length(fxvec)
    fx=fxvec(i1);
    for i2=1:length(fyvec)
        fy=fyvec(i2);
        for i3=1:length(xmaxvec)
            xmax=xmaxvec(i3);
            for i4=1:length(ymaxvec)
                ymax=ymaxvec(i4);
                for i5=1:length(dphixvec)
                    dphix=dphixvec(i5);
                    for i6=1:length(dphiyvec)
                        dphiy=dphiyvec(i6);
                        x_in=0.5+xmax*t_in.*cos(fx*t_in+dphix);
                        y_in=0.5+ymax*sin(fy*t_in+dphiy);
                        txy=[x_in y_in];
                        txydown=downsample(txy,5);
                        nsamp=nsamp+1;
                        allSamples{nsamp}=txy; allDown{nsamp}=txydown;
                    end
                end
            end
        end
    end
end


rmax=0.1;
thetamax=pi/6;
randmax=1000;
fprintf('Initializing training examples\n====================\n');
for j1=1:1000
    if mod(j1-1,randmax/20)==0
       fprintf('=');
    end
    x_in(1)=rand;
    y_in(1)=rand;
    head0=rand*2*pi;
    for j2=2:100
        retry=true;
        while retry
            rrad=rmax*rand;
            headtemp=head0+rand*thetamax;
            xnext=x_in(j2-1)+rrad*cos(headtemp);
            ynext=y_in(j2-1)+rrad*sin(headtemp);
            if(xnext<0 || xnext>1 || ynext<0 || ynext>1)
                retry=true;
            else
                retry=false;
            end
        end
        head0=headtemp;
        x_in(j2)=xnext;
        y_in(j2)=ynext;
    end
    txy=[x_in y_in];
    txydown=downsample(txy,5);
    nsamp=nsamp+1;
    allSamples{nsamp}=txy; allDown{nsamp}=txydown;
end











