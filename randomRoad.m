function [DzDx,z] = randomRoad(x,A_road,Phase_road,n_road)
%RANDOMROAD Compute z and DzDx for a random road
z    = sum(A_road.*sin(n_road*x - Phase_road)) + sum(A_road.*sin(Phase_road));
DzDx = sum(A_road.*n_road.*cos(n_road*x - Phase_road));
end

