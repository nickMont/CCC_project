function omegaR = feedback_linearization_controller(ewxv, omegaR_est, vwind_est, ewxv_des, mode, accel_vec, interr)
j=get_active_quad();

int_errors=zeros(12,1);
if nargin >= 7
    int_errors=interr;
end

%max rotor speed squared
wsquaredmax=100;

feas_step=.95;
feas_step_max_iter=20;

G_body=zeros(6,1);
G_prop=zeros(6,1);
Rm_prop=zeros(6,1);
H_prop=zeros(6,1);
A=zeros(6,1);

[e,wI,x,v]=split_ICs(ewxv,1);
[e_des,wI_des,x_des,v_des]=split_ICs(ewxv_des,1);
qdot=[v;wI];
RBI=calculate_R_from_euler(e);
RIB=RBI';

v_rel_B=RBI*(v-vwind_est);

edot=wI;
xdot=v;
phi=e(1);
theta=e(2);
psi=e(3);


wdir=[1 -1 1 -1];
[m_in, I_rotor_in, ~, rotor_loc_in, J_in, cds_in, ~,~,wdir_in,cls_in]=get_dimensions();
m=m_in(j);
I_rotor=I_rotor_in(j);
rotor_loc=rotor_loc_in(:,:,j);
J=J_in(:,:,j);
cds=cds_in(:,j);
wdir=wdir_in(:,j);
cls=cls_in(:,j);

cH=.001*[1;1;1;1];
cR=.005*[1;1;1;1];
cT=.1*[1;1;1;1];
[~,~,~,kt_set]=get_Motor_Properties();
kT=kt_set*[1;1;1;1];
g=9.81;

W=[1 0 -sin(phi)
   0 cos(phi) sin(phi)*cos(theta)
   0 -sin(phi) cos(phi)*cos(theta)];
Wt=W';
wB=W*wI;

WtJW=Wt*J*W;
if cond(WtJW)>=10^15
    WtJW=WtJW+.1*eye(3);
end
C_mat=C_mat_eval(e,wI,x,v,m,J);
G_mat=[0;0;g;0;0;0];
M_mat=[m*eye(3) zeros(3,3)
    zeros(3,3) WtJW];

G_body=[zeros(3,1)
    Wt*[wB(2)*wB(3)*(J(2,2)-J(3,3))
    wB(1)*wB(3)*(J(3,3)-J(1,1))
    wB(1)*wB(2)*(J(1,1)-J(2,2))]];

G_prop_mat=[zeros(3,4)
    Wt*I_rotor*[wB(2)*[wdir(1) wdir(2) wdir(3) wdir(4)]
    wB(1)*[wdir(1) wdir(2) wdir(3) wdir(4)]
    0 0 0 0]];
G_prop=G_prop_mat*omegaR_est;

A = [RIB*[-sign(v_rel_B(1))*cds(1)*v_rel_B(1)^2
    -sign(v_rel_B(2))*cds(2)*v_rel_B(2)^2
    -sign(v_rel_B(3))*cds(3)*v_rel_B(3)^2+cls(1)*v_rel_B(1)^2+cls(2)*v_rel_B(2)^2]
    Wt*[-sign(wB(1))*wB(1)^2
    -sign(wB(2))*wB(2)^2
    -sign(wB(3))*wB(3)^2]];

% %[cHx1 cHx2 ...
% % cHy1 cHy2 ...
% % cHz1 cHz2 ... ]
cHv=zeros(3,4);
cRv=zeros(3,4);
for i=1:4
    cHv(:,i)=cH(i)*(RBI*v+cross(wB,rotor_loc(:,i)));
    cRv(:,i)=cR(i)*(RBI*v+cross(wB,rotor_loc(:,i)));
end
H_prop_mat = [RIB*-cHv
    Wt*[rotor_loc(3,1)*cHv(2,1)-rotor_loc(1,1)*cHv(3,1) rotor_loc(3,2)*cHv(2,2)-rotor_loc(1,2)*cHv(3,2) rotor_loc(3,3)*cHv(2,3)-rotor_loc(1,3)*cHv(3,3) rotor_loc(3,4)*cHv(2,4)-rotor_loc(1,4)*cHv(3,4)
    rotor_loc(2,1)*cHv(3,1)-rotor_loc(3,1)*cHv(1,1) rotor_loc(2,2)*cHv(3,2)-rotor_loc(3,2)*cHv(1,2) rotor_loc(2,3)*cHv(3,3)-rotor_loc(3,3)*cHv(1,3) rotor_loc(2,4)*cHv(3,4)-rotor_loc(3,4)*cHv(1,4)
    rotor_loc(1,1)*cHv(2,1)-rotor_loc(2,1)*cHv(1,1) rotor_loc(1,2)*cHv(2,2)-rotor_loc(2,2)*cHv(1,2) rotor_loc(1,3)*cHv(2,3)-rotor_loc(2,3)*cHv(1,3) rotor_loc(1,4)*cHv(2,4)-rotor_loc(2,4)*cHv(1,4)]];
H_prop = H_prop_mat*omegaR_est;


Rm_mat=[zeros(3,4)
    Wt*[cRv(1,1)*wdir(1) cRv(1,2)*wdir(2) cRv(1,3)*wdir(3) cRv(1,4)*wdir(4)
    cRv(2,1)*wdir(1) cRv(2,2)*wdir(2) cRv(2,3)*wdir(3) cRv(2,4)*wdir(4)
    cRv(3,1)*wdir(1) cRv(3,2)*wdir(2) cRv(3,3)*wdir(3) cRv(3,4)*wdir(4)]];
Rm_prop=Rm_mat*omegaR_est;

h=C_mat*qdot+G_mat-Rm_prop-H_prop-G_prop-G_body-A;
hu=h(1:2);
hc=h(3:6);


Muu=M_mat(1:2,1:2);
Muc=M_mat(1:2,3:6);
Mcu=M_mat(3:6,1:2);
Mcc=M_mat(3:6,3:6);

B=[RIB*[zeros(2,4)
    1 0 0 0]
    Wt*[0 1 0 0
    0 0 1 0
    0 0 0 1]];
Bu=B(1:2,1:4);
Bc=B(3:6,1:4);


% %gains
% %Tuned gains with error limits fed to PID
% kp_p=3;
% kd_p=1.125*kp_p;
% kI_p=0.035*kp_p;
% xerrmax=10;
% vxerrmax=15;
% yerrmax=xerrmax;
% vyerrmax=vxerrmax;

% %Aggressive but handles poorly with noise of s>20cm
% kp_x=1.4875;
% kd_x=2.255*kp_x;
% kI_x=.1525*kp_x;

% %Tuned for 10m dash with zero latency
% kp_x=1.35;
% kd_x=1.625*kp_x;
% kI_x=.1525*kp_x;

%Tuned for 10m dash with 50ms latency on attitude and 125ms latency on
%position
kp_x=.75;
kd_x=2.4*kp_x;
kI_x=.12*kp_x;

kp_y=1.1;
kd_y=.95*kp_y;
kI_y=.075*kp_y;

xerrmax=200;
vxerrmax=200;
yerrmax=xerrmax;
vyerrmax=vxerrmax;

kp_p_z=7.7;
kd_p_z=1.5*kp_p_z;
kI_p_z=.075*kp_p_z;
kp_a=120;
kd_a=.30*kp_a;

v1=0;
%If acceleration is given as an input
if mode==3
    v1=accel_vec(3);
else  %If acceleration is not specified, use PID
    v1=-kp_p_z*(x(3)-x_des(3)) - kd_p_z*(v(3)-v_des(3)) - kI_p_z*int_errors(9);
end

phi_des=e_des(1);
theta_des=e_des(2);
psi_des=e_des(3);

if mode==1 || mode==3
    
    xerr=x(1)-x_des(1);
    vxerr=v(1)-v_des(1);
    
    yerr=x(2)-x_des(2);
    vyerr=v(2)-v_des(2);
    

    xerr=saturationF(xerr,-xerrmax,xerrmax);
    vxerr=saturationF(vxerr,-vxerrmax,vxerrmax);
    yerr=saturationF(yerr,-yerrmax,yerrmax);
    vyerr=saturationF(vyerr,-vyerrmax,vyerrmax);
    
    xPD=0;
    yPD=0;
    if mode==3
        xPD = accel_vec(1);
        yPD = -accel_vec(2);
    else
        xPD = -kp_x*(xerr) - kd_x*(vxerr) - kI_x*(int_errors(7));
        yPD = -(-kp_y*(yerr) - kd_y*(vyerr) - kI_y*(int_errors(8)));
    end

    theta_des=atan(1/(hc(1)/m+v1)*(cos(psi_des)*(xPD+hu(1)/m) + sin(psi_des)*(yPD+hu(2)/m)));
    phi_des=atan(-cos(theta_des)/(hc(1)/m+v1)*(sin(psi_des)*(xPD+hu(1)/m) - cos(psi_des)*(yPD+hu(2)/m)));
    
    wI_des=zeros(3,1);
    mode=2;
end

if mode==2 || strcmp(mode,'att')
    
    
    v= [v1
        -kp_a*(e(1)-phi_des) - kd_a*(wI(1)-wI_des(1))
        -kp_a*(e(2)-theta_des) - kd_a*(wI(2)-wI_des(2))
        -kp_a*(e(3)-psi_des) - kd_a*(wI(3)-wI_des(3))];
    
    iMuu=inv(Muu);
    delta=(Bc-Mcu*iMuu*Bu);
    alpha=inv(delta)*(Mcc-Mcu*iMuu*Muc);
    Beta=-inv(delta)*(Mcu*iMuu*hu-hc);
    Gamma=alpha*v+Beta;
    
end

%apply saturation to Gamma=[T M1 M2 M3]
%Take saturation limits from body frame [M1 M2 M3]_body, then rotate to
%inertial frame.  Saturate [Gamma(2) Gamma(3) Gamma(4)] in inertial frame
%based on rotated values.  Saturate max thrust.  Then bound rotor speed.
%Saturating body moments is the best way to preserve control inputs.
%Saturating rotors can completely change input torques.

phi_ddot_bodymax=20;
theta_ddot_bodymax=20;
psi_ddot_bodymax=10;

% %Print desired body moments before saturation.
% inv(W')*[Gamma(2);Gamma(3);Gamma(4)]

omegaR_feasible=false;
n=0;
GammaF=Gamma;

ktdm=[cT(1) cT(2) cT(3) cT(4)
    cT(1)*rotor_loc(1,1) cT(2)*rotor_loc(1,2) cT(3)*rotor_loc(1,3) cT(4)*rotor_loc(1,4)
    cT(1)*rotor_loc(2,1) cT(2)*rotor_loc(2,2) cT(3)*rotor_loc(2,3) cT(4)*rotor_loc(2,4)
    kT(1)*cT(1)*wdir(1) kT(2)*cT(2)*wdir(2) kT(3)*cT(3)*wdir(3) kT(4)*cT(4)*wdir(4)];

% Feasibility with v
while omegaR_feasible==false && n<=feas_step_max_iter
    n=n+1;
    
    %adding [.1;.1;.1] to permit turning near gimbal lock
    phithetapsi_inertial_max=Wt*[phi_ddot_bodymax;theta_ddot_bodymax;psi_ddot_bodymax]+[.1;.1;.1];
        
    %adding [.1;.1;.1] to permit turning near gimbal lock
    v(2)=saturationF(v(2),-1*abs(phithetapsi_inertial_max(1)),abs(phithetapsi_inertial_max(1)));
    v(3)=saturationF(v(3),-1*abs(phithetapsi_inertial_max(2)),abs(phithetapsi_inertial_max(2)));
    v(4)=saturationF(v(4),-1*abs(phithetapsi_inertial_max(3)),abs(phithetapsi_inertial_max(3)));
    
    GammaF=alpha*v+Beta;
    
    omegaRsquared=inv(ktdm)*GammaF;
    
    if abs(omegaRsquared)==omegaRsquared
        omegaR_feasible=true;
    end
    
    phi_ddot_bodymax=feas_step*phi_ddot_bodymax;
    theta_ddot_bodymax=feas_step*theta_ddot_bodymax;
    psi_ddot_bodymax=feas_step*psi_ddot_bodymax;
    
end

%omegaR=omegaRsquared;

% %lower bound rotor speed at zero
for i=1:4
    omegaRsquared(i)=saturationF(omegaRsquared(i),0,wsquaredmax);
end

omegaR=omegaRsquared.^0.5;

end


% % Old snippets

% %Try feasibility with Gamma
% while omegaR_feasible==false && n<=feas_step_max_iter
%     n=n+1;
%         
%     %adding [.1;.1;.1] to permit turning near gimbal lock
%     inertial_acceleration_max=abs(W'*[phi_ddot_bodymax;theta_ddot_bodymax;psi_ddot_bodymax]+[.1;.1;.1]);
%     
%     GammaF(2)=saturationF(Gamma(2),-1*inertial_acceleration_max(1),inertial_acceleration_max(1));
%     GammaF(3)=saturationF(Gamma(3),-1*inertial_acceleration_max(2),inertial_acceleration_max(2));
%     GammaF(4)=saturationF(Gamma(4),-1*inertial_acceleration_max(3),inertial_acceleration_max(3));
%     
%     ktdm=[cT(1) cT(2) cT(3) cT(4)
%         cT(1)*rotor_loc(1,1) cT(2)*rotor_loc(1,2) cT(3)*rotor_loc(1,3) cT(4)*rotor_loc(1,4)
%         cT(1)*rotor_loc(2,1) cT(2)*rotor_loc(2,2) cT(3)*rotor_loc(2,3) cT(4)*rotor_loc(2,4)
%         kT(1)*cT(1)*wdir(1) kT(2)*cT(2)*wdir(2) kT(3)*cT(3)*wdir(3) kT(4)*cT(4)*wdir(4)];
%     
%     omegaRsquared=inv(ktdm)*GammaF;
%     
%     if abs(omegaRsquared)==omegaRsquared
%         omegaR_feasible=true;
%     end
%     
%     phi_ddot_bodymax=feas_step*phi_ddot_bodymax;
%     theta_ddot_bodymax=feas_step*theta_ddot_bodymax;
%     psi_ddot_bodymax=feas_step*psi_ddot_bodymax;
%     
% end

