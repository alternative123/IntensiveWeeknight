clear
% start = [0.5,0.5,0.5];
% goal = [1.25 1.25 1.25];
% map = load_map('sample_maps/maptest.txt',0.5,0.5,0.1);
% start = [5,19,3];
% goal = [5,-3,3];
% map = load_map('sample_maps/map0.txt',0.5,0.5,0.1);
start = [0.0  -4.9 0.2];
goal  = [8.0  18.0 3.0];
map = load_map('sample_maps/map0.txt', 0.1, 2.0, 0.3);
% start = [2.4  0.6  3]; % [5,19,3];
% goal = [5,-3,3];
% map = load_map('sample_maps/maptest2.txt',0.5,0.5,0.1);
% start = [2,2.5,3.5];
% goal = [17,2.5,3.5];
% map = load_map('sample_maps/map1.txt',0.2,0.25,0.20);
% start = [1,19,3];
% goal = [9,-3,3];
% map = load_map('sample_maps/map0.txt',0.75,0.75,0.5);
%%
tic
[path, nexpanded]=dijkstra(map,start,goal,false);
fprintf('Got path of length %f with %d nodes expanded in %f seconds\n',sum(sqrt(sum(diff(path).^2,2))),nexpanded,toc);
%%
% astartic = tic;
% [path, nexpanded]=dijkstra(map,start,goal,true);
% fprintf('Got path of length %f with %d nodes expanded in %f seconds\n',sum(sqrt(sum(diff(path).^2,2))),nexpanded,toc(astartic));

free = true(map.gridsize);

n = prod(map.gridsize);
nx = map.gridsize(1);
ny = map.gridsize(2);
nz = map.gridsize(3);

% Offsets to get to center of grid
xoffset = map.xy_res/2;
yoffset = map.xy_res/2;
zoffset = map.z_res/2;

% Helper functions for converting between indeces and 3D coordinates
xindToX = @(x) x*map.xy_res + map.boundary(1,1) - xoffset;
yindToY = @(y) y*map.xy_res + map.boundary(1,2) - yoffset;
zindToZ = @(z) z*map.z_res + map.boundary(1,3) - zoffset;
xyzIndToXYZ = @(x,y,z) [ xindToX(x);
                         yindToY(y);
                         zindToZ(z)  ];

xToXind = @(x) ceil((x - map.boundary(1,1))/map.xy_res);
yToYind = @(y) ceil((y - map.boundary(1,2))/map.xy_res);
zToZind = @(z) ceil((z - map.boundary(1,3))/map.z_res);
xyzToInd = @(x,y,z) xToXind(x) + nx*(yToYind(y)-1) + nx*ny*(zToZind(z)-1);

tic
disp(map.gridsize)
[x,y] = meshgrid(1:map.gridsize(1), 1:map.gridsize(2));
for o = 1:size(map.obstacles,1)
    box = map.obstacles(o,:);
    for z = 1:map.gridsize(3)
        % Calculate 3d positions of the indeces
        p = [...
            xindToX(x(:)),...
            yindToY(y(:)),...
            zindToZ(z*ones(numel(x),1)) ...
            ];
        % Inverse testing
        pinv = [...
            ceil((p(:,1) - map.boundary(1,1))/map.xy_res),...
            ceil((p(:,2) - map.boundary(1,2))/map.xy_res),...
            ceil((p(:,3) - map.boundary(1,3))/map.z_res) ...
            ];
        if ~(all(all(pinv == [x(:) y(:) z*ones(size(x(:)))])))
            disp(pinv(1:12,:))
            disp([x(1:12)' y(1:12)' z*ones(size(x(1:12)'))])
            assert(false)
        end
        % Index testing
        s2is = sub2ind(map.gridsize,x(:),y(:),z*ones(numel(x),1));
        mine = x(:) + nx*(y(:)-1) + nx*ny*(z*ones(numel(x),1)-1);
        assert(all(s2is == mine))
        
        dx = max(max(box(1)-p(:,1),p(:,1)-box(4)),zeros(size(p,1),1));
        dy = max(max(box(2)-p(:,2),p(:,2)-box(5)),zeros(size(p,1),1));
        dz = max(max(box(3)-p(:,3),p(:,3)-box(6)),zeros(size(p,1),1));
        freepoints = ((dx.^2 + dy.^2 + dz.^2) >= map.margin^2);
        free(:,:,z) = free(:,:,z) & ...
            reshape(freepoints,map.gridsize([2 1]))';
    end
end
toc



startXYZ = [xindToX(xToXind(start(1))),yindToY(yToYind(start(2))),zindToZ(zToZind(start(3)))];
goalXYZ = [xindToX(xToXind(goal(1))),yindToY(yToYind(goal(2))),zindToZ(zToZind(goal(3)))];
startNode = xyzToInd(start(1),start(2),start(3));
goalNode = xyzToInd(goal(1),goal(2),goal(3));

plot_path(map,path);
% hold on
% [x,y,z]=ind2sub(map.gridsize,find(~free));
% [x1,y1,z1]=ind2sub(map.gridsize,find(free));
% s = unidrnd(numel(x),450,1);
% s1 = unidrnd(numel(x1),250,1);
% 
% scatter3(...
%     x(s)*map.xy_res+map.boundary(1,1)-xoffset,...
%     y(s)*map.xy_res+map.boundary(1,2)-yoffset,...
%     z(s)*map.z_res+map.boundary(1,3)-zoffset, 'filled')
% 
% hold on
% scatter3(...
%     x1(s1)*map.xy_res+map.boundary(1,1)-xoffset,...
%     y1(s1)*map.xy_res+map.boundary(1,2)-yoffset,...
%     z1(s1)*map.z_res+map.boundary(1,3)-zoffset )
% 
% hold on
% scatter3(...
%     [startXYZ(1) goalXYZ(1)],...
%     [startXYZ(2) goalXYZ(2)],...
%     [startXYZ(3) goalXYZ(3)], ...
%     [ 50 50 ], ...
%     'MarkerEdgeColor','k',...
%     'MarkerFaceColor',[0 .75 .75] )

% localWeights = zeros([3 3 3]);
% localWeights([1 3],[1 3],[1 3]) = sqrt(2*map.xy_res^2+map.z_res^2);
% localWeights([1 3],[1 3],2) = sqrt(2)*map.xy_res;
% localWeights([1 3],2,[1 3]) = sqrt(map.xy_res^2 + map.z_res^2);
% localWeights(2,[1 3],[1 3]) = sqrt(map.xy_res^2 + map.z_res^2);
% localWeights([1 3],2,2) = map.xy_res;
% localWeights(2,[1 3],2) = map.xy_res;
% localWeights(2,2,[1 3]) = map.z_res;
% for x = -1:1
%     for y = -1:1
%     for z = -1:1
%         if any([xi+x,yi+y,zi+z] < 0) || any([xi+x,yi+y,zi+z] > map.gridsize) || ~freespace(xi+x,yi+y,zi+z)
%             continue
%         end
%         cost(xi+x,yi+y,zi+z) = minCost + localWeights(x+2,y+2,z+2);
%         if astar
%             fscore(xi+x,yi+y,zi+z) = cost(xi+x,yi+y,zi+z) + norm(xyzIndToXYZ(xi+x,yi+y,zi+z)-goalXYZ);
%             
%         else
%             fscore(xi+x,yi+y,zi+z) = cost(xi+x,yi+y,zi+z);
%         end
%         
%     end
%     end
%     end


%
% for i = 1:numel(paths)
%     if paths(i) == Inf
%         continue
%     end
%     [xi,yi,zi] = ind2sub(map.gridsize,i);
%     [xn,yn,zn] = ind2sub(map.gridsize,paths(i));
%     if ~(abs(xi-xn) <= 1 && abs(yi-yn) <= 1 && abs(zi-zn) <= 1)
%         fprintf('Failed: %d %d %d vs %d %d %d\n',xi,yi,zi,xn,yn,zn)
%     end
% end

%disp([xi+cx(validindeces) goalXYZ(1)-xindToX(xi+cx(validindeces)) yi+cy(validindeces) goalXYZ(2)-yindToY(yi+cy(validindeces)) zi+cz(validindeces) goalXYZ(3)-zindToZ(zi+cz(validindeces)) heuristic])
%break
