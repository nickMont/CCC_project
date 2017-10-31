function [Aeq,beq] = boundaryCond(n,wp_x,wp_y,wp_z,t_w,n_w)

%Observation: No boundary conditions on initial position and final pos
[Tx0,Tv0,Ta0,Tj0,~] = TimeVectors(0,n);

%Allocate matrices
Aeq_wp_x = zeros(4*n_w,(n_w-1)*(n+1));
Aeq_wp_y = zeros(4*n_w,(n_w-1)*(n+1));
Aeq_wp_z = zeros(4*n_w,(n_w-1)*(n+1));
Aeq_cont = zeros(4*(n_w-2),(n_w-1)*(n+1));

beq_wp_x = zeros(4*n_w,1);
beq_wp_y = zeros(4*n_w,1);
beq_wp_z = zeros(4*n_w,1);
beq_cont = zeros(4*(n_w-2),1);

%First waypoints
range_row = 1:4;
range_col = 1:n+1;
Aeq_wp_x(range_row,range_col) = [Tx0'; Tv0'; Ta0'; Tj0'];
Aeq_wp_y(range_row,range_col) = [Tx0'; Tv0'; Ta0'; Tj0'];
Aeq_wp_z(range_row,range_col) = [Tx0'; Tv0'; Ta0'; Tj0'];
beq_wp_x(range_row,1) = wp_x(:,1);
beq_wp_y(range_row,1) = wp_y(:,1);
beq_wp_z(range_row,1) = wp_z(:,1);

for i = 2:n_w

    [Tx,Tv,Ta,Tj,~] = TimeVectors(t_w(i) - t_w(i-1),n);

    %Enforce waypoints
    range_row = 4*(i-1)+1:4*i;
    range_col = (n+1)*(i-2)+1:(n+1)*(i-1);
    Aeq_wp_x(range_row,range_col) = [Tx'; Tv'; Ta'; Tj'];
    Aeq_wp_y(range_row,range_col) = [Tx'; Tv'; Ta'; Tj'];
    Aeq_wp_z(range_row,range_col) = [Tx'; Tv'; Ta'; Tj'];
    beq_wp_x(range_row,1) = wp_x(:,i);
    beq_wp_y(range_row,1) = wp_y(:,i);
    beq_wp_z(range_row,1) = wp_z(:,i);

    
    %Enforce continuity
    if (i ~= n_w)
        range_row = 4*(i-2)+1:4*(i-1);
        range_col = (n+1)*(i-2)+1:(n+1)*i;
        Aeq_cont(range_row,range_col) = [Tx' -Tx0'; Tv' -Tv0'; Ta' -Ta0'; Tj' -Tj0'];
        beq_cont(range_row,1) = zeros(4,1);
    end
end

%Delete entries with NaN
nan_Index_x = find(isnan(beq_wp_x));
Aeq_wp_x(nan_Index_x,:) = [];
beq_wp_x(nan_Index_x) = [];

nan_Index_y = find(isnan(beq_wp_y));
Aeq_wp_y(nan_Index_y,:) = [];
beq_wp_y(nan_Index_y) = [];

nan_Index_z = find(isnan(beq_wp_z));
Aeq_wp_z(nan_Index_z,:) = [];
beq_wp_z(nan_Index_z) = [];


%Construct equalities
Aeq_x = sparse([Aeq_wp_x; Aeq_cont]);
beq_x = sparse([beq_wp_x; beq_cont]);
Aeq_y = sparse([Aeq_wp_y; Aeq_cont]);
beq_y = sparse([beq_wp_y; beq_cont]);
Aeq_z = sparse([Aeq_wp_z; Aeq_cont]);
beq_z = sparse([beq_wp_z; beq_cont]);

%Construct final matrices
Aeq = blkdiag(Aeq_x,Aeq_y,Aeq_z);
beq = [beq_x; beq_y; beq_z];