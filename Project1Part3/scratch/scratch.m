% Figuring out splines

% T = [ 0 1 2 4 5 6    8 9];
X = [ 0,1,2,3,4, 5, 4, 3, 2, 1, 0,-1,-2,-3,-4,-5,-4,-3,-2,-1,0 ];
Y = [ 5,6,7,8,9,10,11,12,13,14,15,14,13,12,11,10, 9, 8, 7, 6,5 ];
Z = Y;
T = 0:length(X)-1;

path = [ X(1)-0.5 Y(1) Z(1); X(:) Y(:) Z(:) ];


% Optimize path
% If the path is too short, don't optimize it
%
clear pathopt
pathopt = path;
% % Get rid of unnecessary intermediate points on the path
% for i = 3:(size(path,1))
%     % Check if we are on a line
%     a = path(i,:) - pathopt(end,:);
%     b = pathopt(end,:) - pathopt(end-1,:);
% %     fprintf('a=(%f,%f,%f); b=(%f,%f,%f);\n',a(1),a(2),a(3),b(1),b(2),b(3));
% %     fprintf('path(i)=(%f,%f,%f); pathopt(end)=(%f,%f,%f); pathopt(end-1)=(%f,%f,%f)\n',...
% %         path(i,1),path(i,2),path(i,3),...
% %         pathopt(end,1),pathopt(end,2),pathopt(end,3),...
% %         pathopt(end-1,1),pathopt(end-1,2),pathopt(end-1,3));
% %     fprintf('Pass? %d ((%f,%f,%f) vs (%f,%f,%f)) \n',...
% %         all(a/norm(a) == b/norm(b)),...
% %         a(1)/norm(a),a(2)/norm(a),a(3)/norm(a),...
% %         b(1)/norm(b),b(2)/norm(b),b(3)/norm(b));
%     if norm(a/norm(a) - b/norm(b)) < 10^-9
% %         fprintf('here %d\n\n\n',i)
%         pathopt(end,:) = path(i,:);
%     else
% %         fprintf('there %d\n\n\n',i)
%         pathopt = [ pathopt; path(i,:) ];
%     end
% end

scatter(pathopt(:,1),pathopt(:,2),30*(1:size(pathopt,1)),[1 0 0])

% Figure out times automatically
timeend = 14;
pathlengths=sqrt(sum(diff(pathopt).^2,2));
T = [0; cumsum(timeend*(pathlengths/sum(pathlengths)))]';

%

% Spline it up
spline = create_spline(pathopt(:,1),pathopt(:,2),pathopt(:,3),T);

[x,y,z] = spline.vecpos((0:0.1:max(T))');
scatter(X,Y)
hold on
plot(x,y)

