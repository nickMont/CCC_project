function [R,euler] = calculate_R_between_vectors(vA,vB)
%vA = R*vB
%euler=phi,theta,psi

[n1,n2]=size(vA);
n=max(n1,n2);
a=unit_vector(vA);
b=unit_vector(vB);
v=cross(b,a);
s=norm(v);
if s==0
    R=eye(n);
else
    c=dot(a,b);

    vmat=[0 -v(3) v(2)
    v(3) 0 -v(1)
    -v(2) v(1) 0];

    R=eye(n)+vmat+vmat*vmat*(1-c)/s^2;
end


%NOTE: Work out whether +10^-10 or -10^10 based on previous set of
%euler angles
R33=R(3,3);
R22=R(2,2);
if R33==0;
    R33=10^-10;
end
if R22==0
    R22=10^-10;
end
    
phi=asin(-1*R(3,2));
theta=atan(R(3,1)/R33);
psi=atan(R(1,2)/R22);

euler=[phi;theta;psi];


end

