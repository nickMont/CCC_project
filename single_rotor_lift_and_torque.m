function [ Fz_B, Tz_B ] = single_rotor_lift_and_torque(r,w,P,i)
%T = (2*pi*r^2*rho*P^2)^1/3
%^ is a common estimate for propeller thrust based on the assumption that
%the air passing down through the prop loses half of its velocity.  A more
%formal estimate is based on the paper below:
% http://ieeexplore.ieee.org.ezproxy.lib.utexas.edu/stamp/stamp.jsp?tp=&arnumber=4505621&tag=1

%P is input power
%r is the radius of one rotor blade

rho = 1.225; %air density, can set global later
z = [0; 0; 1];

%Propeller torque provides gyroscopic effect
[~,~,~,Kt] = get_Motor_Properties;

%Thrust is based on induced velocity, which does not change if wind is
%blowing up or down.  Air velocity is only a disturbance.
Thrust = thrust_magnitude(P,r,i);  %magnitude of thrust
Fz_B = Thrust*z;
wdir = w/norm(w);  %Torque is parallel to angular velocity
Tz_B = Kt*Thrust*-wdir;  %Tz_B is the torque contribution of gyroscopic stiffness


end

