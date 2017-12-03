function [F,T] = calculate_body_lift_and_torque(t,e_w_x_v_0)

%i is a 4x1 vector of electrical current at time t
[i,~] = elec_current(t);

%Only need w to calculate direction of rotor torque
[w,alpha,P] = rotor_profile(t,e_w_x_v_0);

%Calculate lift and torque of each rotor
[~,I_rotor,r_rotor,rotor_locations,~] = get_dimensions;
[F1,T1] = single_rotor_lift_and_torque(r_rotor,w(:,1),P(1),i(1));
[F2,T2] = single_rotor_lift_and_torque(r_rotor,w(:,2),P(2),i(2));
[F3,T3] = single_rotor_lift_and_torque(r_rotor,w(:,3),P(3),i(3));
[F4,T4] = single_rotor_lift_and_torque(r_rotor,w(:,4),P(4),i(4));

%T1-T4 are the torques of the rotors, opposite in direction to the torques
%induced on the body by their spin.
torque_rotor_1 = alpha(:,1)*I_rotor + cross(rotor_locations(:,1),F1) - T1;
torque_rotor_2 = alpha(:,2)*I_rotor + cross(rotor_locations(:,2),F2) - T2;
torque_rotor_3 = alpha(:,3)*I_rotor + cross(rotor_locations(:,3),F3) - T3;
torque_rotor_4 = alpha(:,4)*I_rotor + cross(rotor_locations(:,4),F4) - T4;

T = torque_rotor_1 + torque_rotor_2 + torque_rotor_3 + torque_rotor_4;
F = F1+F2+F3+F4;

end

