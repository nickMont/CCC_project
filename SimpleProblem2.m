clc;
clear all;
close all;

numset=10:1:100-1;
tictoc=zeros(length(numset),1);
mcmax=1;

%If running old control
% load('SimulationParameters.mat');
%If running Mellinger control
% load('MellingerSimulationParameters.mat');

for mcl=1:mcmax
    mcl
    for ij=1:length(numset)
        
        tic
        
        %  Parameters
        n = 7;              %Polynomial order
        t_w = 0:2*pi/numset(ij):2*pi;      %Time to reach each waypoint
        n_w = numel(t_w);   %Number of waypoints
        
        %Conditions on x
        x_w = cos(t_w);   %Position x at each waypoint
        vx_w = [0 NaN(numset(ij)-1,1)' 0];
        ax_w = [0 NaN(numset(ij)-1,1)' 0];
        jx_w = [0 NaN(numset(ij)-1,1)' 0];
        
        %Conditions on y
        y_w = sin(t_w);   %Position y at each waypoint
        vy_w = [0 NaN(numset(ij)-1,1)' 0];
        ay_w = [0 NaN(numset(ij)-1,1)' 0];
        jy_w = [0 NaN(numset(ij)-1,1)' 0];
        
        %Conditions on z
        z_w = zeros(numset(ij)+1,1)';   %Position z at each waypoint
        vz_w = [0 NaN(numset(ij)-1,1)' 0];
        az_w = [0 NaN(numset(ij)-1,1)' 0];
        jz_w = [0 NaN(numset(ij)-1,1)' 0];
        
        Waypoints.x = [x_w; vx_w; ax_w; jx_w];
        Waypoints.y = [y_w; vy_w; ay_w; jy_w];
        Waypoints.z = [z_w; vz_w; az_w; jz_w];
        Waypoints.t = t_w;
        
        SolCoeff = solveMinSnap(Waypoints);
        
        tt=toc;
        tictoc(ij)=tictoc(ij)+tt/mcmax;
    end
end
dt_subsamp=0.05;
[px,py,pz]=processSolCoeff(t_w,SolCoeff,dt_subsamp);
figure(1); clf;
plot(numset(5:end)+1,tictoc(5:end)+1)
xlabel('Number of points along trajectory')
ylabel('Rescaled time to optimize')






