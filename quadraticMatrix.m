function [H] = quadraticMatrix(n,t_w,n_w)

%Time vectors
% syms t real;
% for i = 0:n
%     Cs(i+1,1) = i*(i-1)*(i-2)*(i-3)*t^(i-4);
% end
% 
% %Matrix H
% H = [];
% Hred = [];
% for i = 1:n_w-1
%     H_new = int(Cs*Cs',t,t_w(i),t_w(i+1));
%     H = blkdiag(H, H_new);
%     Hred = blkdiag(Hred, H_new(5:end,5:end));
% end
% H2 = double(H);
% Hred = double(Hred);

H = sparse((n_w-1)*(n+1),(n_w-1)*(n+1));

for i = 0:n
    Cj(i+1).coeff = i*(i-1)*(i-2)*(i-3);
    Cj(i+1).exp = max(0,i-4);
end

for k = 1:n_w-1
    for i = 1:n+1
        for j = 1:n+1
            exp = Cj(i).exp + Cj(j).exp+1;
            coeff = (Cj(i).coeff*Cj(j).coeff)/exp;
            integral_monomial = coeff*((t_w(k+1) - t_w(k))^exp);
            H((k-1)*(n+1)+i,(k-1)*(n+1)+j) = integral_monomial;
        end
    end
end


% H = Tinv'*H*Tinv;