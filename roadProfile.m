function [DzDx, z] = roadProfile(x, z0_road, lambda_road, w_road, d_bump)
%ROADPROFILE Returns z and DzDx for a given distance

% Road profile for a bump
% [DzDx, z] = roadBump(x, z0_road, lambda_road, w_road, d_bump);

% Flat and sinusoidal road
[DzDx, z] = roadSin(x, z0_road, lambda_road, w_road, d_bump);

% Flat road
% DzDx = 0;
% z = 0;
end