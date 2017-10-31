function [solcoeffs,J] = callMinSnap2D(L1)
t_w=L1(:,1)';
nn=length(t_w);

x_w=L1(:,2)';
y_w=L1(:,3)';
z_w=zeros(1,21);

vx_w=[0 NaN(1,nn-2) 0];
vy_w=[0 NaN(1,nn-2) 0];
vz_w=zeros(1,21);

ax_w=[0 NaN(1,nn-2) 0];
ay_w=[0 NaN(1,nn-2) 0];
az_w=zeros(1,21);

jx_w=[0 NaN(1,nn-2) 0];
jy_w=[0 NaN(1,nn-2) 0];
jz_w=zeros(1,21);

Waypoints.x = [x_w; vx_w; ax_w; jx_w];
Waypoints.y = [y_w; vy_w; ay_w; jy_w];
Waypoints.z = [z_w; vz_w; az_w; jz_w];
Waypoints.t = t_w;

[solcoeffs,J] = solveMinSnap(Waypoints);


end

