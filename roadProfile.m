function [DzDx, z] = roadProfile(x,z0_road,lambda_road,w_road,d_bump,...
    A_road,Phase_road,n_road)
%ROADPROFILE Returns z and DzDx for a given distance

% Road profile for a bump
% [DzDx, z] = roadBump(x, z0_road, lambda_road, w_road, d_bump);

% Flat and sinusoidal road
% [DzDx, z] = roadSin(x, z0_road, lambda_road, w_road, d_bump);

% Random road
% [DzDx,z] = randomRoad(x,A_road,Phase_road,n_road);

% Flat road
DzDx = 0;
z = 0;
end