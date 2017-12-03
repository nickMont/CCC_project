function q = calculate_quaternion_from_R(R)
%http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19990052720.pdf
tra=trace(R);
[mx,i]=max([R(1,1) R(2,2) R(3,3), tra]);



if i==1
    q_notnorm=[2*mx+1-tra; R(1,2)+R(2,1); R(1,3)+R(3,1); R(2,3)-R(3,2)];
    q=q_notnorm/norm(q_notnorm);
elseif i==2
    q_notnorm=[R(2,1)+R(1,2); 2*mx+1-tra; R(2,3)+R(3,2); R(3,1)-R(1,3)];
    q=q_notnorm/norm(q_notnorm);
elseif i==3
    q_notnorm=[R(3,1)+R(1,3); R(3,2)+R(2,3); 2*mx+1-tra; R(1,2)-R(2,1)];
    q=q_notnorm/norm(q_notnorm);
elseif i==4
    q_notnorm=[R(2,3)-R(3,2); R(3,1)-R(1,3); R(1,2)-R(2,1); 1+tra];
    q=q_notnorm/norm(q_notnorm);
else
    q=[9001;9001;9001;9001];
end

end

