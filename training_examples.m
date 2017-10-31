x_in=0.5+xmax*t_in.*cos(2*pi*t_in);
y_in=0.5+ymax*sin(2*pi*t_in);
txy=[x_in y_in];
txydown=downsample(txy,5);
allSamples{1}=txy; allDown{1}=txydown;

x_in=0.5+xmax*t_in.*cos(2*pi*t_in+pi/2);
y_in=0.5+ymax*sin(2*pi*t_in+pi/2);
txy=[x_in y_in];
txydown=downsample(txy,5);
allSamples{2}=txy; allDown{2}=txydown;

x_in=0.5+xmax*t_in.*cos(2*pi*t_in+pi/4);
y_in=0.5+ymax*sin(2*pi*t_in+pi/4);
txy=[x_in y_in];
txydown=downsample(txy,5);
allSamples{3}=txy; allDown{3}=txydown;

x_in=-0.5+xmax*t_in.*cos(2*pi*t_in);
y_in=0.5+ymax*sin(2*pi*t_in);
txy=[x_in y_in];
txydown=downsample(txy,5);
dt_down=max(t_in)/(length(txydown)-1);
t_samp=(0:dt_down:max(t_in))';
allSamples{4}=txy; allDown{4}=txydown;

x_in=-0.5+xmax*t_in.*cos(2*pi*t_in+pi/2);
y_in=0.5+ymax*sin(2*pi*t_in+pi/2);
txy=[x_in y_in];
txydown=downsample(txy,5);
allSamples{5}=txy; allDown{5}=txydown;

x_in=-0.5+xmax*t_in.*cos(2*pi*t_in+pi/4);
y_in=0.5+ymax*sin(2*pi*t_in+pi/4);
txy=[x_in y_in];
txydown=downsample(txy,5);
allSamples{6}=txy; allDown{6}=txydown;