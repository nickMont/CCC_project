function [SolCoeff,Cost] = solveMinSnap(Waypoints)

n_w = numel(Waypoints.t);
n = 7;  %This solution only uses 7th or 9th order polynomials
% tic;
%Quadratic matrix
H = quadraticMatrix(n,Waypoints.t,n_w);
% t1 = toc
% tic
%Boundary conditions enforcement
[Aeq,beq] = boundaryCond(n,Waypoints.x,Waypoints.y,Waypoints.z,Waypoints.t,n_w);
% [Aeq_y,beq_y] = boundaryCond(n,Waypoints.y,Waypoints.t,n_w);
% [Aeq_z,beq_z] = boundaryCond(n,Waypoints.z,Waypoints.t,n_w);

%Transformation Matrix
Tinv = transformationMatrix(n,Waypoints.t,n_w);

%Build optimization problem
H2 = Tinv'*H*Tinv;
Hbig = blkdiag(H2,H2,H2);
% Aeq = blkdiag(Aeq_x*Tinv, Aeq_y*Tinv, Aeq_z*Tinv);
Aeq = Aeq*blkdiag(Tinv,Tinv,Tinv);
% beq = [beq_x; beq_y; beq_z];

[nr,nc] = size(Aeq);

M = [Hbig Aeq'; Aeq sparse(nr,nr)];
Y = [sparse(nc,1); beq];
Sol = M\Y;
SolBc = (Sol(1:nc)); 
SolCoeff = blkdiag(Tinv,Tinv,Tinv)*SolBc;
% t2 = toc
Cost = SolBc'*Hbig*SolBc;