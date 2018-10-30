function [DzDx, z] = roadSin(x, z0_road, ~, w_road, d_bump)
%ROADPROFILE Returns z and DzDx for a given distance to obtain a sinusoidal
%road

if x < d_bump
    z = 0;
    DzDx = 0;
else
    z = z0_road/2 * (1-cos(w_road*(x-d_bump)));
    DzDx = z0_road/2 * w_road * sin(w_road*(x-d_bump));
end

