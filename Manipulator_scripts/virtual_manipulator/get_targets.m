function [target_cell] = get_targets(len, type)
% Get the data for the earth/planet positions.
% 


skiprow = 0;    
target_cell = {};
if type == 0  % This is to get the array for the earth pointing

    path = './data/earth_pos_data.txt';
    
    data = readtable(path);
    i = 1;
    while size(target_cell, 2) < len
        
        point = [data.sc_2_earth_LVLH_1(i+skiprow); data.sc_2_earth_LVLH_2(i+skiprow); data.sc_2_earth_LVLH_3(i+skiprow)];
        new_point = point;
        target_cell{i} = new_point;
        i= i + 1;
    end
end

if type == 1

    path = './data/MARS_LVLH.mat';
    z = load(path);
    i = 1;
    while size(target_cell, 2) < len
        
        point = [0; 0; z.MARS_LVLH(i)];
        new_point = point;
        target_cell{i} = new_point;
        i= i + 1;
    end

end


end














