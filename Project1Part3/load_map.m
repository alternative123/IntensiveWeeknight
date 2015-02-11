function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered full if it lies within 'margin' distance of
%  on obstacle.

% Create map structure
map.boundary = zeros(2,3);
map.obstacles = [];
map.obstacle_colors = [];
map.xy_res = xy_res;
map.z_res = z_res;
map.margin = margin;

% Open map file and parse to create environment
fid = fopen(filename);

tline = fgetl(fid);
while ischar(tline)
    C = mystrsplit(tline); % TODO: Replace this with MATLAB 2014b's
    tline = fgetl(fid);

    if isempty(C{1}) || C{1}(1) == '#'
        continue
    end
    
    % Actual parsing
    if strcmp(C{1},'boundary')
        map.boundary(1,:) = [ str2num(C{2}) str2num(C{3}) str2num(C{4}) ];
        map.boundary(2,:) = [ str2num(C{5}) str2num(C{6}) str2num(C{7}) ];
    elseif strcmp(C{1},'block')
        map.obstacles = [ ...
            map.obstacles;
            str2double(C{2}) str2double(C{3}) str2double(C{4}) str2double(C{5}) str2double(C{6}) str2double(C{7}) ];
        if length(C) >= 8
            map.obstacle_colors = [ ...
                map.obstacle_colors;
                str2double(C{8}) str2double(C{9}) str2double(C{10}) ];
        else
            map.obstacle_colors = [ map.obstacle_colors; 0 0 0 ];
        end
    else
        disp('I do not know what is happening...')
    end
end

fclose(fid);

% Create graph
% Size of x, y, and z dimensions
sizex = ceil((map.boundary(2,1) - map.boundary(1,1)) / map.xy_res);
sizey = ceil((map.boundary(2,2) - map.boundary(1,2)) / map.xy_res);
sizez = ceil((map.boundary(2,3) - map.boundary(1,3)) / map.z_res);

map.gridsize = [sizex,sizey,sizez];

% disp([sizex,sizey,sizez])

% map.freespace = true(map.gridsize);
% 
% for o = 1:size(map.obstacles,1)
%     
% end

end






