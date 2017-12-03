function q = euler_to_quaternion(EA_vec)
%NOTE: uses 3-2-1 rotation.  Will have to recode if doing 3-1-3
q =[cos(EA_vec(1)/2)*cos(EA_vec(2)/2)*cos(EA_vec(3)/2)+sin(EA_vec(1)/2)*sin(EA_vec(2)/2)*sin(EA_vec(3)/2)
    sin(EA_vec(1)/2)*cos(EA_vec(2)/2)*cos(EA_vec(3)/2)-cos(EA_vec(1)/2)*sin(EA_vec(2)/2)*sin(EA_vec(3)/2)
    cos(EA_vec(1)/2)*sin(EA_vec(2)/2)*cos(EA_vec(3)/2)+sin(EA_vec(1)/2)*cos(EA_vec(2)/2)*sin(EA_vec(3)/2)
    cos(EA_vec(1)/2)*cos(EA_vec(2)/2)*sin(EA_vec(3)/2)-sin(EA_vec(1)/2)*sin(EA_vec(2)/2)*cos(EA_vec(3)/2)];

q = quat_con(q);

end

