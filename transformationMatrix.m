function Tinv = transformationMatrix(n,t_w,n_w)

%Transformation for the H matrix
[Tx0,Tv0,Ta0,Tj0,Ts0] = TimeVectors(0,n);
T_w = sparse(n+1,n+1);
% T = zeros((n_w-1)*(n+1),(n_w-1)*(n+1));
Tinv = zeros((n_w-1)*(n+1),(n_w-1)*(n+1));



for i = 1:n_w-1
    dt = t_w(i+1) - t_w(i);
    if(dt >= 1)
        [Tx,Tv,Ta,Tj,Ts] = TimeVectors(dt,n);
        if(n == 7)
            T_w = sparse([Tx0'; Tv0'; Ta0'; Tj0'; Tx'; Tv'; Ta'; Tj']);
        elseif(n == 9)
            T_w = sparse([Tx0'; Tv0'; Ta0'; Tj0'; Ts0'; Tx'; Tv'; Ta'; Tj'; Ts']);
        end
        range = (i-1)*(n+1)+1:i*(n+1);
        Tinv(range,range) = inv(T_w);
    else
        range = (i-1)*(n+1)+1:i*(n+1);
        Tinv(range,range) = eye(n+1);
    end

%     T(range,range) = T_w;
%     Tinv(range,range) = inv(T_w);
%     Tinv(range,range) = eye(n+1);
%     logCond(i) = log10(condest(inv(T_w)));
end
% logCond
Tinv = sparse(Tinv);