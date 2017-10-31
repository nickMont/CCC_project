function [Tx,Tv,Ta,Tj,Ts] = TimeVectors(t,n)

Tx = zeros(n+1,1);
for i = 0:n
    Tx(i+1) = t^i;
end

Tv = zeros(n+1,1);
for i = 1:n
    Tv(i+1) = i*t^(i-1);
end

Ta = zeros(n+1,1);
for i = 2:n
    Ta(i+1) = i*(i-1)*t^(i-2);
end

Tj = zeros(n+1,1);
for i = 3:n
    Tj(i+1) = i*(i-1)*(i-2)*t^(i-3);
end

Ts = zeros(n+1,1);
for i = 4:n
    Ts(i+1) = i*(i-1)*(i-2)*(i-3)*t^(i-4);
end