function [DzDx, z] = roadBump(x, z0_road, lambda_road, w_road, d_bump)
%ROADPROFILE Returns z and DzDx for a given distance

% Road profile for a bump
% Compute z
if x < d_bump
    z = 0;
    DzDx = 0;
elseif x < d_bump + lambda_road
    z = z0_road/2 * (1-cos(w_road*(x-d_bump)));
    DzDx = z0_road/2 * w_road * sin(w_road*(x-d_bump));
else
    z = 0;
    DzDx = 0;
end

