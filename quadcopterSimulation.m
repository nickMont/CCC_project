%Quadcopter dynamics simulator
%UT Austin RNL
clear;clc;
%close all;
digits(32)


load nmats_circle

makeGifFlag=0; %1 to make gif, 0 else
gifname='dumpgif.gif';

plotperformanceFlag=0;
plotQuadFlag=1;

%Will loop through quads->objects->walls
numquads=1;

rescalefactor=5;
%PID position gains, calmer
kp_x = 1.39;
kd_x = 1.17;
kI_x = 0.19;
kf_ff= 0.165;
kp_y=kp_x;
kI_y=kI_x;
kd_y=kd_x;
kp_p_z=1.35;
kd_p_z=2.75*kp_p_z;
kI_p_z=0;
dontUseZeroVel=1; %estimate velocity if ==1, try to stop if ==0

veclim=1*ones(12,1);
%basic params
t0=0;
t_max=t0+20;
f=@(x) tanh(x);%sigmoid or tanh
subint=1;
dt_down=1;
uprate=20; %rate of control generation 

% % %draw trajectory
figure(42);clf;
axis([0 1 0 1])
lenTraj=101;
h=imfreehand();
xyTraj=h.getPosition();
xyTrajSubsamp=resample(xyTraj,lenTraj,length(xyTraj));
close(42)

t_samp=(0:dt_down:max(t_max))';
L1=f(nmat1')*xyTrajSubsamp;
L2=f(nmat2')*L1;
[solCoeff,~]=callMinSnap2D([t_samp L2],t_samp,subint);
[xop,yop]=processSolCoeff(t_samp,solCoeff,1/uprate);
xop=rescalefactor*xop; yop=rescalefactor*yop;
% load xyOp_t20v2 %loaded with scalefactors



%Simulation time
t_int = 1/uprate; %time interval between steps
plot_int = .002;
plottime=t_max; %final time value to print in main simulation graphic
num_cycles_latency_e = 0 * ones(numquads,1);  %50ms max @40Hz control generation
num_cycles_latency_pos= 0 * ones(numquads,1); %125ms max @40Hz control generation

s_noise_e = pi/180 * 0;  %attitude noise, input deg
s_noise_x = .01 * 0;  %position noise, input cm
s_noise_v = s_noise_x/2;


%Starting point and destination
%Euler angles ordered as phi, theta, psi (roll, pitch, yaw)
e_w_x_v_prev(:,1,1) = [0;0;0; 0;0;0; xop(1);yop(1);0; (xop(2)-xop(1))/t_int;(yop(2)-yop(1))/t_int;0];

e_w_x_v_des = zeros(12,numquads);
e_w_x_v_des(:,1)=[0;0;0; 0;0;0; xop(2);yop(2);0; 0;0;0];

% waypoints=[e_w_x_v_des1 e_w_x_v_des2];  %
% %set_destination_point(waypoints(:,1));


pos_close=false;
angle_close=false;
waypoint_number=1;
%[~,max_waypoints]=size(waypoints);
%e_w_x_v_des = waypoints(:,waypoint_number);


F_control_max=50;
F_control_min=-10;
min_thrust=1;
max_thrust=1000;
set_f_bounds([F_control_min;F_control_max],[min_thrust;max_thrust]);

%Moments of inertia about various axes
Ixx = 0.35;
Iyy = 0.35;
Izz = 0.45;
Ixy = 0;
Ixz = 0;
Iyz = 0;
J=ones(3,3,numquads);
J(:,:,1) = [Ixx Ixy Ixz
    Ixy Iyy Iyz
    Ixz Iyz Izz];
if numquads>=2
    for i=2:numquads
        J(:,:,i)=J(:,:,1);
    end
end

m = 2.5*ones(1,numquads); %kg

%Physical properties of rotors (all assumed identical)
I_rotor = .02*ones(1,numquads); %moment of inertia of a single rotor
r_rotor = .05*ones(1,numquads);
r_dis = 0.25; %distance between cg of quad and rotor center

cds = [.1;.1;.2]*ones(1,numquads); %Drag coefficients in body frame for velocity directed along each axis
%ie cds(1) = cd_x, or the drag coefficient corresponding to a velocity in x
cls = [.01;.01]*ones(1,numquads);
c_pqr=[0;0;0]*ones(1,numquads);

cross_area = [.2;.2,;.5]*ones(1,numquads);
%Projected cross-sectional area from a viewpoint along each axis

%Rotor locations as vectors in body frame:
%    ^      (nose)
% 4     1
%    x
% 3     2
%                   |
% . is origin, -x-, y 
%                   |
r1loc = r_dis*unit_vector([1;1;0]);
r2loc = r_dis*unit_vector([1;-1;0]);
r3loc = r_dis*unit_vector([-1;-1;0]);
r4loc = r_dis*unit_vector([-1;1;0]);
rotor_loc=zeros(3,4,numquads);
rotor_loc(:,:,1)=[r1loc r2loc r3loc r4loc];
if numquads>=2
    for i=2:numquads
        rotor_loc(:,:,i)=rotor_loc(:,:,1);
    end
end

wdir=[1 -1 1 -1]'*ones(1,numquads);

set_dimensions(m, I_rotor, r_rotor, rotor_loc, J, cds, cross_area,r_dis,wdir,cls,c_pqr);
%NOTE how axes are rendered with y going from +1 to -1.  It can be
%disorienting, make sure to check coordinates of engine in the plot before
%calling something as incorrect


%Set motor properties
%~,~,~,kT
set_Motor_Properties(.7,.5,.7,1.5)
%See code to see which call corresponds to which coefficient/update later

Ts = 10; %constant of how quickly wind speed changes
Tv = .2; %constant of how quickly wind direction changes
s0 = 2; %base windspeed
d0 = [.9;.1;0]; %unit vector of base wind direction
set_Air_Properties(1.225,Ts,Tv,s0,d0)


%Initial state
%q_0 = [0;0;0;1];  %body frame initially aligned with inertial frame
% e = phi,theta,psi or roll, pitch, yaw

x = e_w_x_v_prev(7:9,1,1);  %notation, for plotting consistency

% jvec=[1,1+1j,1-1j]';
% e_gain=5;
% w_gain=1;
% x_gain=4;
% v_gain=2;
% poles = [-e_gain*jvec -w_gain*jvec -x_gain*jvec -v_gain*jvec];

% %Set up integrator backstepping
%set_IB_gains(c_e,c_w,c_Int);
%integral_error = [0;0;0;0;0;0];  %integrated error for backstepping controller
%last_error_info = [0;0;0;0;0;0]; %communicate whether last time step's error had the 
% %same sign as this time step's error to determine whether or not to flush it


circpts=zeros(3,50,numquads);
t_circ=linspace(0,2*pi,50);
for j=1:numquads
    for i=1:50
        circpts(:,i,j)=r_rotor(j)*[cos(t_circ(i));sin(t_circ(i));0];
    end
end

figure(1)
clf;

interr_entries=5;
lastpts_errors=zeros(12,interr_entries,j);

arenabounds=[-1 rescalefactor+1 -1 rescalefactor+1 -1 1];

statehist=zeros(12,1,numquads);

if plotQuadFlag==1
for j=1:numquads
    ewxv_j=e_w_x_v_prev(:,1,j);
    e=ewxv_j(1:3);
    x=ewxv_j(7:9);
    RBI = calculate_R_from_euler(e);  %rotation matrix to B from I
    RIB = RBI';  %rotate to I from B, ie vI = RIB * vB
    xB_I = RIB*[1;0;0]; %rotate unit x axis in B to be displayed in I
    yB_I = RIB*[0;1;0];
    zB_I = RIB*[0;0;1];

    figure(1)
    %     x_body = plot3([x(1) x(1)+xB_I(1)], [x(2) x(2)+xB_I(2)], [x(3) x(3)+xB_I(3)], 'blue');
    %     hold on
    %     y_body = plot3([x(1) x(1)+yB_I(1)], [x(2) x(2)+yB_I(2)], [x(3) x(3)+yB_I(3)], 'green');
    %     hold on
    %     z_body = plot3([x(1) x(1)+zB_I(1)], [x(2) x(2)+zB_I(2)], [x(3) x(3)+zB_I(3)], 'red');
    hold on
    view(3)
    rotor1_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,1,j)*ones(1,50));
    rotor1plot(j) = plot3(rotor1_circle(1,:), rotor1_circle(2,:), rotor1_circle(3,:), 'red');
    hold on
    rotor2_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,2,j)*ones(1,50));
    rotor2plot(j) = plot3(rotor2_circle(1,:), rotor2_circle(2,:), rotor2_circle(3,:), 'red');
    hold on
    rotor3_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,3,j)*ones(1,50));
    rotor3plot(j) = plot3(rotor3_circle(1,:), rotor3_circle(2,:), rotor3_circle(3,:), 'blue');
    hold on
    rotor4_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,4,j)*ones(1,50));
    rotor4plot(j) = plot3(rotor4_circle(1,:), rotor4_circle(2,:), rotor4_circle(3,:), 'blue');
    axis(arenabounds)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    
    
end  %plot quads
end

frametime=.15;
if makeGifFlag==1
    fcounter=0; %framecounter
    fcounter=fcounter+1;
    frame=getframe(1);
    im=frame2im(frame);
    [imind,cm]=rgb2ind(im,256);
    if fcounter==1
        imwrite(imind,cm,gifname,'gif','Loopcount',inf,'DelayTime',frametime);
    else
        imwrite(imind,cm,gifname,'gif','WriteMode','append','DelayTime',frametime);
    end
end

omegaR_vec=zeros(4,j);

omegaHist=zeros(4,10,numquads);

n=0;
for i = t0:t_int:t_max %note: i will be time
    n=n+1;
    %    %Set desired angles to zero if initializing controller
    %     if n==0
    %        e_w_x_v_des(1:3)=zeros(1,3);
    %     end
    %
    pause(plot_int)
      
    
    e_w_x_v_estimated=zeros(12,numquads);
    %Add noise to all quads
    for j=1:numquads
        w_noise = [0;0;0];
        v_noise = [0;0;0];
        e_noise = s_noise_e*eye(3)*randn(3,1);
        x_noise = s_noise_x*eye(3)*randn(3,1);
        v_noise = s_noise_v*eye(3)*randn(3,1);
        noisevec(:,n,j)=[x_noise;e_noise];
        %Latency of attitude
        if n-num_cycles_latency_e(j)<=0
            e_w_x_v_estimated(1:6,j)=e_w_x_v_prev(1:6,1,j);
        else
            e_w_x_v_estimated(1:6,j) = e_w_x_v_prev(1:6,n-num_cycles_latency_e(j),j);
        end
        %Latency of position
        if n-num_cycles_latency_pos(j)<=0
            e_w_x_v_estimated(7:12,j)=e_w_x_v_prev(7:12,1,j);
        else
            e_w_x_v_estimated(7:12,j) = e_w_x_v_prev(7:12,n-num_cycles_latency_pos(j),j);
        end
        e_w_x_v_estimated(:,j) = e_w_x_v_estimated(:,j) + [e_noise;w_noise;x_noise;v_noise]; %add noise
    end
    
%     xvpvazk=zeros(20,numquads);
%     wall_null=[0;0;0;1337;0;0;0;0;0];
%     point_null=[42;42;42;0;0;0;0;0;0;0;0;0];
%     for j=1:numquads
%         xvpvazk(:,j)=[e_w_x_v_estimated(7:12,j);zeros(9,1);2;6;10;10;5];
%     end
%     derp=mediationLayer(xvpvazk)
    
    for j=1:numquads
        set_active_quad(j)
        
        %     delete(x_body)
        %     delete(y_body)
        %     delete(z_body)
        
        
        %can use clf; instead of delete but I like having the lingering visual
        %after the simulation is done running
        
        t = i;  %for clarity
        t_ode = [t, t+t_int]; %t is t_0, t+t_int is tf
        
        
        % %Develop control action
        %e_noise = [randn*s_noise(1); randn*s_noise(2); randn*s_noise(3)]; %simulate noise
        
        xcirc=xop(n);
        ycirc=yop(n);
        %zcirc=.25*i;
        zcirc=0;
        if n>1 && n<200
            xdotcirc=dontUseZeroVel*(-xop(n)+xop(n+1))/t_int;
            ydotcirc=dontUseZeroVel*(-yop(n)+yop(n+1))/t_int;
            FF=kf_ff*10^-3*0.5*[(xop(n)+xop(n+2))/t_int^2;(yop(n)+yop(n+1))/t_int^2;0];
            %FF=[0;0;0];
        elseif n==1
            xdotcirc=xop(n+1)/t_int;
            ydotcirc=yop(n+1)/t_int;
            FF=[0;0;0];
        else
            xdotcirc=0;
            ydotcirc=0;
            FF=[0;0;0];
        end
        xdotcircprev=xdotcirc;
        ydotcircprev=ydotcirc;
        %zdotcirc=.25;
        zdotcirc=0;
        e_w_x_v_des(:,j)=[0;0;0; 0;0;0; xcirc;ycirc;zcirc; xdotcirc;ydotcirc;zdotcirc];
        
        d_ewxv_est =  -(e_w_x_v_des(:,j) - e_w_x_v_estimated(:,j));
        %[A_lin,B_lin] = lin_A_ewxv_SF(t, d_ewxv);  %NOTE: returns B from MT
        
        for k=1:interr_entries-1
            lastpts_errors(:,k,j)=lastpts_errors(:,k+1,j);
        end
        lastpts_errors(:,interr_entries,j)=d_ewxv_est;
 
        errint=(sum((lastpts_errors(:,:,j)')))';
        errint=vectorSaturationF(errint,veclim);
        
        
        %Signs are opposite of values shown elsewhere in code due to
        %negations within FL_controller
        xaccel=-kp_x*(d_ewxv_est(7)) - kd_x*(d_ewxv_est(10)) - kI_x*(errint(7)) + FF(1);
        yaccel=(-kp_y*(d_ewxv_est(8)) - kd_y*(d_ewxv_est(11)) - kI_y*(errint(8))) + FF(2);
        zaccel=-kp_p_z*(d_ewxv_est(9)) - kd_p_z*(d_ewxv_est(12)) - kI_p_z*errint(9) + FF(3);
        
%        kd_x=0;
%        kp_x=10;  %safer at 5/.1
%        kI_x=.5;
%        xaccel=-kp_x*(d_ewxv_est(10)) - kd_x*(d_ewxv_est(10)) - kI_x*(errint(10));%FF velocity control on x
        
        PIDscaling=1;
        
        accelvec=PIDscaling*[xaccel;yaccel;zaccel];
        
        omegaR_vec_q=feedback_linearization_controller(e_w_x_v_estimated(:,j),omegaR_vec(:,j),[0;0;0],e_w_x_v_des(:,j),3,accelvec, errint);
        %omegaR_vec_q=feedback_linearization_controller(e_w_x_v_estimated(:,j),[0;0;0;0],[0;0;0],e_w_x_v_des(:,j),1)
        omegaR_vec(:,j)=omegaR_vec_q;
        set_control_omegaR(omegaR_vec_q);
        omegaHist(:,n,j)=omegaR_vec_q;
        
        %Propagate actual system
        %Calculation of the best control input is done here too
        %[~, e_w_x_v_t] = ode23s('e_w_x_v_dot', t_ode, e_w_x_v_prev);
        [tvec, e_w_x_v_t] = ode45('eI_wI_x_v_dot', t_ode, e_w_x_v_prev(:,n,j));
        
        %use ode23s if stiffness errors appear
        
        %Split up end results
        e_w_x_v_last = e_w_x_v_t(end,:);  %gets state of last time step
        e = e_w_x_v_last(1:3)';
        w = e_w_x_v_last(4:6)';
        x = e_w_x_v_last(7:9)';
        v = e_w_x_v_last(10:12)';
        e_w_x_v_prev(:,n+1,j) = [e;w;x;v];
        %n=0 on first iteration, need second column equal to output
        ewxv_vec(:,n+2,j)=e_w_x_v_prev(:,n,j);
        statehist(:,n,j)=e_w_x_v_prev(:,n,j);
    end
    
    if plotQuadFlag==1
    if i<=plottime
        for j=1:numquads
            delete(rotor1plot(j))
            delete(rotor2plot(j))
            delete(rotor3plot(j))
            delete(rotor4plot(j))
        end  %plot quads
        for j=1:numquads
            ewxv_j=e_w_x_v_prev(:,n,j);
            e=ewxv_j(1:3);
            x=ewxv_j(7:9);
            RBI = calculate_R_from_euler(e);  %rotation matrix to B from I
            RIB = RBI';  %rotate to I from B, ie vI = RIB * vB
            xB_I = RIB*[1;0;0]; %rotate unit x axis in B to be displayed in I
            yB_I = RIB*[0;1;0];
            zB_I = RIB*[0;0;1];
            
            if makeGifFlag==1
                figure(1)
            end %force to plot on fig1 if a gif is being made
            %     x_body = plot3([x(1) x(1)+xB_I(1)], [x(2) x(2)+xB_I(2)], [x(3) x(3)+xB_I(3)], 'blue');
            %     hold on
            %     y_body = plot3([x(1) x(1)+yB_I(1)], [x(2) x(2)+yB_I(2)], [x(3) x(3)+yB_I(3)], 'green');
            %     hold on
            %     z_body = plot3([x(1) x(1)+zB_I(1)], [x(2) x(2)+zB_I(2)], [x(3) x(3)+zB_I(3)], 'red');
            hold on
            view(3)
            rotor1_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,1,j)*ones(1,50));
            rotor1plot(j) = plot3(rotor1_circle(1,:), rotor1_circle(2,:), rotor1_circle(3,:), 'red');
            hold on
            rotor2_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,2,j)*ones(1,50));
            rotor2plot(j) = plot3(rotor2_circle(1,:), rotor2_circle(2,:), rotor2_circle(3,:), 'red');
            hold on
            rotor3_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,3,j)*ones(1,50));
            rotor3plot(j) = plot3(rotor3_circle(1,:), rotor3_circle(2,:), rotor3_circle(3,:), 'blue');
            hold on
            rotor4_circle=x*ones(1,50)+RIB*(circpts(:,:,j)+rotor_loc(:,4,j)*ones(1,50));
            rotor4plot(j) = plot3(rotor4_circle(1,:), rotor4_circle(2,:), rotor4_circle(3,:), 'blue');
            axis(arenabounds)
            xlabel('X')
            ylabel('Y')
            zlabel('Z')
            grid on
            if n>=5
                scatter3(ewxv_vec(7,n),ewxv_vec(8,n),ewxv_vec(9,n),'g.');
            end
            
        end
    end
    end
    
    if makeGifFlag==1
        fcounter=fcounter+1;
        frame=getframe(1);
        im=frame2im(frame);
        [imind,cm]=rgb2ind(im,256);
        if fcounter==1
            imwrite(imind,cm,gifname,'gif','Loopcount',inf,'DelayTime',frametime);
        else
            imwrite(imind,cm,gifname,'gif','WriteMode','append','DelayTime',frametime);
        end
    end
    %Group outputs here for ease of deletion
    
    %
    
    
end
kvec=linspace(0,t_max,n);

if plotperformanceFlag==1
    for j=1:numquads
        figure(2*j)
        clf;
        xdesvec=e_w_x_v_des(7:9,j);
        x_xdes=xdesvec(1)*ones(n);
        y_xdes=xdesvec(2)*ones(n);
        z_xdes=xdesvec(3)*ones(n);
        plot(kvec,180/pi*ewxv_vec(1,3:end,j),'b')
        hold on
        plot(kvec,180/pi*ewxv_vec(2,3:end,j),'r')
        hold on
        plot(kvec,180/pi*ewxv_vec(3,3:end,j),'g')
        hold off
        axis([0 t_max -90 90])
        title('Attitude')
        legend('roll','pitch','yaw')
        xlabel('Time (s)')
        ylabel('Attitude (deg)')
        
        %Position
        %actual pos
        figure(2*j+1)
        clf;
        plot(kvec,ewxv_vec(7,3:end,j),'b')
        hold on
        plot(kvec,ewxv_vec(8,3:end,j),'r')
        hold on
        plot(kvec,ewxv_vec(9,3:end,j),'g')
        %desired pos
        hold on
        plot(kvec,x_xdes,':b')
        hold on
        plot(kvec,y_xdes,':r')
        hold on
        plot(kvec,z_xdes,':g')
        legend('x','y','z')
        %axis([0 tmax min(arenabounds)-1 max(arenabounds)+1])
        axis([0 t_max -5 15])
        title('Position')
        xlabel('Time (s)')
        ylabel('Distance from origin (m)')
    end
end

% figure(2*numquads+2)
% clf;
% hold on
% plot(kvec,noisevec(1,1:end,1)*10^2,'-b')
% hold on
% plot(kvec,noisevec(2,1:end,1)*10^2,'-r')
% hold on
% plot(kvec,noisevec(3,1:end,1)*10^2,'-g')
% title('NoiseHist position')
% 
% figure(2*numquads+3)
% clf;
% hold on
% plot(kvec,noisevec(4,1:end,1)*10^3,':b')
% hold on
% plot(kvec,noisevec(5,1:end,1)*10^3,':r')
% hold on
% plot(kvec,noisevec(6,1:end,1)*10^3,':g')
% title('NoiseHist attitude')

% figure(87)
% clf;
% plot(kvec,ewxv_vec(7,3:end,1),'b')
% hold on
% plot(kvec,ewxv_vec(7,3:end,2),'r')
% hold on
% xdesvec=e_w_x_v_des(7:9,1);
% x_xdes=xdesvec(1)*ones(n);
% plot(kvec,x_xdes,':b')
% hold on
% xdesvec=e_w_x_v_des(7:9,2);
% x_xdes=xdesvec(1)*ones(n);
% plot(kvec,x_xdes,':r')
% legend('x1','x2','x1des','x2des')
% axis([0 tmax -1 11])

% figure(2)
% view(3)
% plot3(e_w_x_v_prev(7,:,1),e_w_x_v_prev(8,:,1),e_w_x_v_prev(9,:,1))
figure(3);clf;
plot(ewxv_vec(7,3:end,j),ewxv_vec(8,3:end,j),'r')
hold on
plot(xop,yop)
axis([0 rescalefactor*1.5 0 rescalefactor*1.5])



