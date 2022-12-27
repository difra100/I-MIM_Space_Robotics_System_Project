function [target_cell] = get_targets(len, type)
% Get the data for the earth/planet positions.
% 


skiprow = 0;    
target_cell = {};
if type == 'earth'  % This is to get the array for the earth pointing

    R = [0 0 1;  % This rotation brings the LVLH frame to the Inertia frame.
        0 -1 0;
        1 0 0];

    path = './data/earth_pos_data.txt';
    
    data = readtable(path);
    i = 1;
    while size(target_cell, 2) < len
        
        point = [data.sc_2_earth_LVLH_1(i+skiprow); data.sc_2_earth_LVLH_2(i+skiprow); data.sc_2_earth_LVLH_3(i+skiprow)];
        new_point = R*point;
        target_cell{i} = new_point;
        i= i + 1;
    end
end


end














