function [a,phi] = calculate_vector_from_quaternion(q)

q = q/norm(q);
phi = 2*acos(q(4));
a = q(1:3) / sin(phi/2);
a = a/norm(a); %normalize to a unit vector

end

