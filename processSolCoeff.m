function [p_x,p_y,p_z] = processSolCoeff(t_w,SolCoeff,dt)
%processes output of minimum snap code into xyz coordinates
time = t_w(1):dt:t_w(end);
n_w=numel(t_w); %numwpts
n = 7; %order
sol_a = reshape(SolCoeff,[n+1 3*(n_w-1)]);
[nr,nc] = size(sol_a); %#ok
sol_ax = sol_a(:,1:nc/3);
sol_ay = sol_a(:,nc/3+1:2*nc/3);
sol_az = sol_a(:,2*nc/3+1:nc);
k = 1;
for i = 1:numel(time)
    if (time(i) > t_w(k+1))
        k = k+1;
    end
    [Tx,Tv,Ta,Tj,Ts] = TimeVectors(time(i)-t_w(k),n); %#ok<ASGLU>
    p_x(i) = Tx'*sol_ax(:,k);
    p_y(i) = Tx'*sol_ay(:,k);
    p_z(i) = Tx'*sol_az(:,k);
end


end

