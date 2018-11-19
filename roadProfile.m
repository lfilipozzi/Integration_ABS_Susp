function [DzDx, z] = roadProfile(x,z0_road,lambda_road,w_road,d_bump,...
    A_road,Phase_road,n_road,roadProfile_index)
%ROADPROFILE Returns z and DzDx at a given distance x for a road profile

switch roadProfile_index
    case 0
        % Flat road
        DzDx = 0;
        z = 0;
    case 1
        % Road profile for a bump
        [DzDx, z] = roadBump(x, z0_road, lambda_road, w_road, d_bump);
    case 2
        % Sinusoidal road
        [DzDx, z] = roadSin(x, z0_road, lambda_road, w_road, d_bump);
    case 3
        % Random road
        [DzDx,z] = randomRoad(x,A_road,Phase_road,n_road);
    otherwise
        DzDx = 0;
        z = 0;
end

end