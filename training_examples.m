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
