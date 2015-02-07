tic
map = load_map('map1.txt',0.1,2.0,0.5);
toc
start = [0.0  -4.9 0.2];
stop  = [8.0  18.0 3.0];
tic
path = dijkstra(map, start, stop, true);
toc
init_script
test_trajectory(start, stop, map, path)
