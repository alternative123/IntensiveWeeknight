function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.



hold on
drawwirebox(map.boundary(2,:),map.boundary(1,:))

for i = 1:size(map.obstacles,1)
    p = map.obstacles(i,1:3);
    q = map.obstacles(i,4:6);
    c = map.obstacle_colors(i,:)/255;
    drawbox(p,q,c);
end

if all(size(path) > 0)
    plot3(path(:,1),path(:,2),path(:,3))
    scatter3(path(:,1),path(:,2),path(:,3))
end

axis image;
view(3);

xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');

end

function drawbox(p,q,c)
% p is top point, q is bottom point, c is color
hold on
% Thank you mathworks
% Top face
fill3([p(1) q(1) q(1) p(1)],...
      [p(2) p(2) q(2) q(2)],...
      [p(3) p(3) p(3) p(3)],c);

% Bottom face:
fill3([p(1) q(1) q(1) p(1)],...
      [p(2) p(2) q(2) q(2)],...
      [q(3) q(3) q(3) q(3)],c);

% Other faces:
X=[p(1) q(1) q(1) p(1) p(1);
   p(1) q(1) q(1) p(1) p(1)];

Y=[p(2) p(2) q(2) q(2) p(2);
   p(2) p(2) q(2) q(2) p(2)];

Z=[p(3) p(3) p(3) p(3) p(3);
   q(3) q(3) q(3) q(3) q(3)];

hs=surf(X,Y,Z);
set(hs,'facecolor',c);

end

function drawwirebox(p,q)
% p is top point, q is bottom point, c is color
hold on
% Thank you mathworks
% All the faces:
X=[p(1) q(1) q(1) p(1) p(1);
   p(1) q(1) q(1) p(1) p(1)];

Y=[p(2) p(2) q(2) q(2) p(2);
   p(2) p(2) q(2) q(2) p(2)];

Z=[p(3) p(3) p(3) p(3) p(3);
   q(3) q(3) q(3) q(3) q(3)];

hs=surf(X,Y,Z);
set(hs,'facecolor','none');

end


