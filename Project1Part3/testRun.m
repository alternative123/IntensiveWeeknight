% map = load_map('maps/map1.txt',0.1,2.0,0.3);
% start = [0.0  -4.9 0.2];
% % start = [5.0  -4.9 3.5];
% stop  = [8.0  18.0 3.0]; % map 1

% map = load_map('maps/map2.txt',0.1,2.0,0.4);
% start = [0.0  -4.9 0.2];
% stop = [10.0 29 3.0]; % map 2

% map = load_map('maps/map3.txt',0.1,2.0,0.3);
% start = [2 2.5 5];
% stop  = [17  2.5 5]; % map 1

map = load_map('maps/map_test1.txt',0.1,5.0,0.1);
start = [0 0 0];
stop = [7.0 15.2 3.0]; % map 2

tic
path = dijkstra(map, start, stop, true);
toc
%
% plot_path(map,optimize_path(path,map))
%

init_script
test_trajectory(start, stop, map, path);
