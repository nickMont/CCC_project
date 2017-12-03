function v = vectorSaturationF(v,vlim)
j=length(v);
for i=1:j
    if v(i)<-vlim(i)
        v(i)=vlim(i);
    elseif v(i)>vlim(i)
        v(i)=vlim(i);
    end
end
end

