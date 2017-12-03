function MT_control = attitude_MT_IB(ewxv, del_e, int_errors,T)
%phi,theta,psi

%[~,I_r,~,~,J]=get_dimensions();
[m,I_r,r_rotor,rotor_loc,J,~,~] = get_dimensions;
dm = norm(rotor_loc(:,1));
[~,~,~,Kt] = get_Motor_Properties;
g = 9.81;

Ixx = J(1,1);
Iyy = J(2,2);
Izz = J(3,3);
b1 = 1/Ixx;
b2 = 1/Iyy;
b3 = 1/Izz;
a1 = (Iyy-Izz)/Ixx;
a2 = I_r/Ixx;
a3 = (Izz-Ixx)/Iyy;
a4 = I_r/Iyy;
a5 = (Ixx-Iyy)/Izz;


del_e = -del_e;  %per sign convention of Full Control of a Quadrotor
chi_1 = int_errors(1); %integral tracking error
chi_2 = int_errors(2);
chi_3 = int_errors(3);
e1 = del_e(1);
w1 = del_e(4);
e2 = del_e(2);
w2 = del_e(5);
e3 = del_e(3);
w3 = del_e(6);


%The simpler approximation
c_p=50;
c_d=.5*c_p;
c_i=0;
%
ce1p=c_p;
ce1d=c_d;
ce1i=c_i;

ce2p=c_p;
ce2d=c_d;
ce2i=c_i;

ce3p=c_p;
ce3d=c_d;
ce3i=c_i;
%Approximate phidotdot setpoints
ddphi_dtdt = sign(e1)*sign(w1)*.1;
ddtheta_dtdt = sign(e2)*sign(w2)*.1;
ddpsi_dtdt = sign(e3)*sign(w3)*.1;

omegaR = 0; %approximate rotor speed

M1 = 1/b1*(ce1p*e1 + ce1d*w1 - ce1i*chi_1 + ddphi_dtdt - ewxv(5)*ewxv(6)*a1 - ewxv(4)*a2*omegaR);
M2 = 1/b2*(ce2p*e2 + ce2d*w2 - ce2i*chi_2 + ddtheta_dtdt - ewxv(4)*ewxv(6)*a3 - ewxv(4)*a4*omegaR);
M3 = 1/b3*(ce3p*e3 + ce3d*w3 - ce3i*chi_3);

% %gains
% c1 = .2;
% c4 = .3;
% c2 = .2;
% c5 = .1;
% c3 = .1;
% c6 = .1;
% lambda1 = .1;  %controls strength of IB feedback
% lambda2 = .1;
% lambda3 = .1;

% %The right way
% [c_e,c_w,c_Int] = get_IB_gains;
% %Note: Gains should be >1
% c1 = c_e(1);
% c2 = c_e(2);
% c3 = c_e(3);
% c4 = c_w(1);
% c5 = c_w(2);
% c6 = c_w(3);
% lambda1 = c_Int(1);
% lambda2 = c_Int(2);
% lambda3 = c_Int(3);
% 
% 
% %Approximate phidotdot setpoints
% ddphi_dtdt = sign(e1)*sign(w1)*.1;
% ddtheta_dtdt = sign(e2)*sign(w2)*.1;
% ddpsi_dtdt = sign(e3)*sign(w3)*.1;
% 
% omegaR = 0; %approximate rotor speed
% 
% M1 = 1/b1*((1-c1^2+lambda1)*e1 + (c1+c4)*w1 - c1*lambda1*chi_1 + ddphi_dtdt - ewxv(5)*ewxv(6)*a1 - ewxv(4)*a2*omegaR);
% M2 = 1/b2*((1-c2^2+lambda2)*e2 + (c2+c5)*w2 - c2*lambda2*chi_2 + ddtheta_dtdt - ewxv(4)*ewxv(6)*a3 - ewxv(4)*a4*omegaR);
% M3 = 1/b3*((1-c3^2+lambda3)*e3 + (c3+c6)*w3 - c3*lambda3*chi_3);


if nargin == 3 %hover if called with three arguments, produce desired T if called with 4
    RBI = calculate_R_from_euler(ewxv(1:3));
    z_I=[0;0;1];
    z_B = RBI * z_I;
    theta = acos(dot(z_B,z_I));
    
    %avoid division by zero
    if theta==pi/2
        theta = pi/2 - .001;
    elseif theta == pi/2
        theta = -pi/2+.001;
    end
    
    T_h = m*g/cos(theta);
else
    T_h = T;
end


MT_control = [M1; M2; M3; T_h];
%Approximate limits on rotor forces
M_bound=get_info();
%M_bound = 2;
T_bound = 100;
for i=1:3
    if MT_control(i) > M_bound
        MT_control(i) = M_bound;
    elseif MT_control(i) < -M_bound
        MT_control(i) = -M_bound;
    end
end
if MT_control(4)>T_bound
    MT_control(4)=T_bound;
end



% F_to_MT = F_to_MT_mat(Kt,dm);
% forces_required = real(inv(F_to_MT)*MT_control);

end

