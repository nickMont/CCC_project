function [orientation,w,x,v] = split_ICs(input_condition, mode)

if mode==1 || mode =='e' || mode =='E'
    e = input_condition(1:3);
    orientation = e;
    w = input_condition(4:6);
    x = input_condition(7:9);
    v = input_condition(10:12);
elseif mode ==2 || mode == 'q' || mode == 'Q'
    %Separate 13-element state and normalize q
    q = quat_con( input_condition(1:4) ); %note--Matlab will not enforce quaternion constraint between timesteps so it must be done manually
    orientation = q;
    w = input_condition(5:7);
    x = input_condition(8:10);
    v = input_condition(11:13);
else
    orientation=9001;
    w=9001;
    x=9001;
    v=9001;
end

end

