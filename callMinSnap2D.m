function [solcoeffs,J] = callMinSnap2D(L1,t_w,subint)
nn=length(t_w);
lenL1=length(L1)

if(subint==1)
    x_w=L1(:,2)';
    y_w=L1(:,3)';
    z_w=zeros(1,21);
else
    x_w=NaN(1,nn); y_w=NaN(1,nn); z_w=zeros(1,nn);
    size(x_w)
    x_w(1)=L1(1,2);
    y_w(2)=L1(1,3);
    for ij=2:lenL1-1
        x_w(1+ij*subint)=L1(ij,2);
        y_w(1+ij*subint)=L1(ij,3);
    end
    x_w(end)=L1(end,2);
    y_w(end)=L1(end,3);
end

vx_w=[0 NaN(1,nn-2) 0];
vy_w=[0 NaN(1,nn-2) 0];
vz_w=zeros(1,nn);

ax_w=[0 NaN(1,nn-2) 0];
ay_w=[0 NaN(1,nn-2) 0];
az_w=zeros(1,nn);

jx_w=[0 NaN(1,nn-2) 0];
jy_w=[0 NaN(1,nn-2) 0];
jz_w=zeros(1,nn);

Waypoints.x = [x_w; vx_w; ax_w; jx_w];
Waypoints.y = [y_w; vy_w; ay_w; jy_w];
Waypoints.z = [z_w; vz_w; az_w; jz_w];
Waypoints.t = t_w;

[solcoeffs,J] = solveMinSnap(Waypoints);


end

