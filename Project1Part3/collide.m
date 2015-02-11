function [C] = collide(map, p)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
C = false(size(p,1),1);

% Magic formula provided by stack overflow, see post titled:
% calculating distance between a point and a rectangular box (nearest point)
for o = 1:size(map.obstacles,1)
    box = map.obstacles(o,:);
    dx = max(max((box(1))-p(:,1),p(:,1)-(box(4))),zeros(size(p,1),1));
    dy = max(max((box(2))-p(:,2),p(:,2)-(box(5))),zeros(size(p,1),1));
    dz = max(max((box(3))-p(:,3),p(:,3)-(box(6))),zeros(size(p,1),1));
    C = C | ((dx.^2 + dy.^2 + dz.^2) < (map.margin).^2);
end


end
