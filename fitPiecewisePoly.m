function [traj] = fitPiecewisePoly(n,n_piece,time,X)

%polynomial order = n
%number of pieces = n_piece
%fit X = X(time) as a set of piecewise polynomials

len = numel(time);

%Break time into chunks
for i = 1:n_piece
    init_piece(i) = floor((i-1)*len/n_piece + 1);
    end_piece(i) = floor(i*len/n_piece);
end

for i = 1:n_piece
    t2 = time(init_piece(i):end_piece(i));
    X2 = X(init_piece(i):end_piece(i));
    p = polyfit(t2,X2,n);
    if(i == n_piece)
        traj.t0(i) = time(init_piece(i));
        traj.tf(i) = time(end_piece(i));
    else
        traj.t0(i) = time(init_piece(i));
        traj.tf(i) = time(init_piece(i+1));
    end
    traj.polyCoeff(:,i) = p';
end