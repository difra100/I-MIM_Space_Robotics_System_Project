function [target_cell] = get_targets(start, len, type)
% Get the data for the earth/planet positions.
% INPUTs: start is the row at which the data collection is meant to start,
% len: The number of row for the experiments, type: This is the planet
% definition {0: Earth, 1: Mars}.

skiprow = 0; % This can be modifed to evaluate different time delays between configurations.


target_cell = {};
if type == 0  % This is to get the array for the earth pointing
    % Earth data is expressed in form of table
    path = './data/earth_pos_data.txt';
    
    data = readtable(path);
    i = 1;
    while size(target_cell, 2) < len
        
        point = [data.sc_2_earth_LVLH_1(start+skiprow+i); data.sc_2_earth_LVLH_2(start+skiprow+i); data.sc_2_earth_LVLH_3(start+skiprow+i)];
        new_point = point;
        target_cell{i} = new_point;
        i= i + 1;
    end
end

if type == 1
    % Mars in LVLH is on the z-axis, thus it is only provided its modulus
    % in the z direction.
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














