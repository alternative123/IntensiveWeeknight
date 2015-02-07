tic
% map = load_map('maps/map1.txt',0.1,2.0,0.3);
map = load_map('maps/map2.txt',0.1,2.0,0.3);
toc
start = [0.0  -4.9 0.2];
% stop  = [8.0  18.0 3.0]; % map 1
stop = [10.0 29 3.0]; % map 2
tic
path = dijkstra(map, start, stop, true);
toc

% plot_path(map,path)


init_script
test_trajectory(start, stop, map, path)
