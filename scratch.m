clear;clc;

load nmats

makeGifFlag=0; %1 to make gif, 0 else
gifname='dumpgif.gif';

plotperformanceFlag=0;

%Will loop through quads->objects->walls
numquads=1;

%PID position gains, calmer
kp_x=.65;  %.75
kd_x=2.95*kp_x;
kI_x=.25*kp_x;
kp_y=.35;  %1.1
kd_y=.95*kp_y;
kI_y=.075*kp_y;
kp_p_z=2.2;
kd_p_z=1.5*kp_p_z;
kI_p_z=.075*kp_p_z;


%draw trajectory
figure(42);clf;
axis([0 1 0 1])
lenTraj=101;
h=imfreehand();
xyTraj=h.getPosition();
xyTrajSubsamp=resample(xyTraj,lenTraj,length(xyTraj));

t_in=5;
f=@(x) tanh(x);%sigmoid or tanh
subint=1;
dt_down=0.25;
t_samp=(0:dt_down:max(t_in))';
L1=f(nmat1')*xyTrajSubsamp;
L2=f(nmat2')*L1;
[solCoeff,~]=callMinSnap2D([t_samp L2],t_samp,subint);
[xop,yop]=processSolCoeff(t_samp,solCoeff,0.05);
[xop' yop']-xyTrajSubsamp




