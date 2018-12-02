function roadProfile_index = getRoadIndex(roadProfile_name)
%GETROADINDEX Give the corresponding index to the road profile for Simulink
%simulation.
%   Since it is impossible to give a string as an input to a MATLAB
%   function in Simulink, the road profile is converted into an index road
%   profile which is then used in Simulink.
    switch roadProfile_name
        case 'flat'
            roadProfile_index = 0;
            disp('Road profile: flat')
        case 'bump'
            roadProfile_index = 1;
            disp('Road profile: bump')
        case 'sinusoidal'
            roadProfile_index = 2;
            disp('Road profile: sinusoidal')
        case 'random'
            roadProfile_index = 3;
            disp('Road profile: random')
        otherwise
            error(['Unknowns road profile "', roadProfile_name, ...
                '". Please select one value among: "flat", "bump", "sinusoidal", and "random".'])
    end
end