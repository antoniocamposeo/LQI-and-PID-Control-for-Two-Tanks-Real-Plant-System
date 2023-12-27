function [G] = get_plant_fdt(A,B,Ts)
G = tf(B,A,Ts);
end

