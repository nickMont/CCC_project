%CODE BORROWED FROM MARCELINO ALMEDIAN

clc;
clear all;
close all;

%If running old control
% load('SimulationParameters.mat');
%If running Mellinger control
% load('MellingerSimulationParameters.mat');

%  Parameters

%Generate simple trajectory
tf = 1*pi;

t_w = 0:0.1:tf;
n_w = numel(t_w);   %Number of waypoints

x_w = cos(.2*t_w);
y_w = sin(2*t_w);
z_w = zeros(1,numel(t_w));

%Stablish initial and final conditions as 0 (the rest is NaN - no
%constraint)
m = numel(t_w);
vx_w = nan(1,m); vy_w = nan(1,m); vz_w = nan(1,m);
ax_w = nan(1,m); ay_w = nan(1,m); az_w = nan(1,m);
jx_w = nan(1,m); jy_w = nan(1,m); jz_w = nan(1,m);
vx_w(1) = 0; vx_w(end) = 0;
vy_w(1) = 0; vy_w(end) = 0;
vz_w(1) = 0; vz_w(end) = 0;
ax_w(1) = 0; ax_w(end) = 0;
ay_w(1) = 0; ay_w(end) = 0;
az_w(1) = 0; az_w(end) = 0;
jx_w(1) = 0; jx_w(end) = 0;
jy_w(1) = 0; jy_w(end) = 0;
jz_w(1) = 0; jz_w(end) = 0;
vx_w(end) = nan;
vy_w(end) = nan;
vz_w(end) = nan;
ax_w(end) = nan;
ay_w(end) = nan;
az_w(end) = nan;
jx_w(end) = nan;
jy_w(end) = nan;
jz_w(end) = nan;

Waypoints.x = [x_w; vx_w; ax_w; jx_w];
Waypoints.y = [y_w; vy_w; ay_w; jy_w];
Waypoints.z = [z_w; vz_w; az_w; jz_w];
Waypoints.t = t_w;

% Gradient descent
% tic;
% [SolCoeff,Cost] = solveMinSnap(Waypoints);
% toc;


tic;
[SolCoeff,Cost,t_w] = minSnapGradientDescent(Waypoints); %#ok<ASGLU>
toc

Waypoints.t = t_w;
[SolCoeff,CurCost] = solveMinSnap(Waypoints);
Cost;
t_w;


% Plot solution
n = 7;
sol_a = reshape(SolCoeff,[n+1 3*(n_w-1)]);
[nr,nc] = size(sol_a);
sol_ax = sol_a(:,1:nc/3);
sol_ay = sol_a(:,nc/3+1:2*nc/3);
sol_az = sol_a(:,2*nc/3+1:nc);
dt = 0.01;

time = t_w(1):dt:t_w(end);
k = 1;
for i = 1:numel(time)
    if (time(i) > t_w(k+1))
        k = k+1;
    end

    [Tx,Tv,Ta,Tj,Ts] = TimeVectors(time(i)-t_w(k),n);
    p_x(i) = Tx'*sol_ax(:,k);
    v_x(i) = Tv'*sol_ax(:,k);
    a_x(i) = Ta'*sol_ax(:,k);
    j_x(i) = Tj'*sol_ax(:,k);
    s_x(i) = Ts'*sol_ax(:,k);
    p_y(i) = Tx'*sol_ay(:,k);
    v_y(i) = Tv'*sol_ay(:,k);
    a_y(i) = Ta'*sol_ay(:,k);
    j_y(i) = Tj'*sol_ay(:,k);
    s_y(i) = Ts'*sol_ay(:,k);
    p_z(i) = Tx'*sol_az(:,k);
    v_z(i) = Tv'*sol_az(:,k);
    a_z(i) = Ta'*sol_az(:,k);
    j_z(i) = Tj'*sol_az(:,k);
    s_z(i) = Ts'*sol_az(:,k);
end
% 
% X_Pr = [time' p_x']; X_Vr = [time' v_x']; X_Ar = [time' a_x'];
% Y_Pr = [time' p_y']; Y_Vr = [time' v_y']; Y_Ar = [time' a_y'];
% Z_Pr = [time' p_z']; Z_Vr = [time' v_z']; Z_Ar = [time' a_z'];
% 
% PhiPr = [time' zeros(numel(time),1)]; 
% PhiVr = [time' zeros(numel(time),1)]; 
% PhiAr = [time' zeros(numel(time),1)]; 
% ThetaPr = [time' zeros(numel(time),1)]; 
% ThetaVr = [time' zeros(numel(time),1)]; 
% ThetaAr = [time' zeros(numel(time),1)]; 
% PsiPr = [time' zeros(numel(time),1)]; 
% PsiVr = [time' zeros(numel(time),1)]; 
% PsiAr = [time' zeros(numel(time),1)]; 
% 
% cd_ini = zeros(6,1);
% c_ini = [p_x(1); p_y(1); p_z(1); 0; 0; 0];
% 
% time2 = time + time(end);
% ModeSim = [time(1:end-2)'  2*ones(numel(time)-2,1)];
% % save('minSnapTraj.mat','xr','yr','zr','vxr','vyr','vzr','axr','ayr','azr');
% 
% figure(1); 
% subplot(5,1,1);plot(time,p_x); hold on;
% subplot(5,1,2);plot(time,v_x); hold on;
% subplot(5,1,3);plot(time,a_x); hold on;
% subplot(5,1,4);plot(time,j_x); hold on;
% subplot(5,1,5);plot(time,s_x); hold on;
% 
% figure(2); 
% subplot(5,1,1);plot(time,p_y); hold on;
% subplot(5,1,2);plot(time,v_y); hold on;
% subplot(5,1,3);plot(time,a_y); hold on;
% subplot(5,1,4);plot(time,j_y); hold on;
% subplot(5,1,5);plot(time,s_y); hold on;
% 
% figure(3); 
% subplot(5,1,1);plot(time,p_z); hold on;
% subplot(5,1,2);plot(time,v_z); hold on;
% subplot(5,1,3);plot(time,a_z); hold on;
% subplot(5,1,4);plot(time,j_z); hold on;
% subplot(5,1,5);plot(time,s_z); hold on;
% 
% figure(4); plot3(p_x,p_y,p_z); hold on; grid on;
% axislim = [min(p_x)-0.5 max(p_x)+0.5 min(p_y)-0.5 max(p_y)+0.5 min(p_z)-0.5 max(p_z)+0.5];
% for i = 1:n_w
% %     [x_w(i),y_w(i),z_w(i)]
%     plot3(x_w(i),y_w(i),z_w(i),'r*'); 
% %     axis(axislim);
% end
% % xlabel('x'); ylabel('y'); zlabel('z');