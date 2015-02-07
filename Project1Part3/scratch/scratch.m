% Figuring out splines

T = [0 1 2 4 5 6    8 9];
X = [1 2 3 4 3 2   -1 1];
Y = [2 3 1 3 5 4.5  3 2];
Z = [2 3 1 3 5 4.5  3 2];

spline = create_spline(X,Y,Z,T);

% n = length(T); % Number of points
% m = 3+1; % Order of the polynomial
% % Number of position constraints: 
% A = zeros(m*(n-1));
% bx = zeros(m*(n-1),1);
% by = zeros(m*(n-1),1);
% bz = zeros(m*(n-1),1);
% 
% 
% % Positions
% for i = 1:n-1
%     j = i+1;
%     % Coefficients matrix
%     A(2*i-1,(1:m)+m*(i-1)) = [T(i)^3 T(i)^2 T(i) 1];
%     A(2*i  ,(1:m)+m*(i-1)) = [T(j)^3 T(j)^2 T(j) 1];
% 
%     % b for the Ax = b for the x,y,z parts
%     bx(2*i-1) = X(i);
%     bx(2*i)   = X(j);
%     by(2*i-1) = Y(i);
%     by(2*i)   = Y(j);
%     bz(2*i-1) = Y(i);
%     bz(2*i)   = Y(j);
% end
% 
% % Velocities at the boundaries
% offset = 2*(n-1);
% for i = 1:n-2
%     j = i+1;
%     % Coefficients matrix
%     A(i+offset,(1:m)+m*(i-1)) = [3*T(j)^2 2*T(j) 1 0];
%     A(i+offset,(1:m)+m*(i)) = -[3*T(j)^2 2*T(j) 1 0];
% end
% 
% % Accelerations at the boundaries
% offset = 2*(n-1)+(n-2);
% for i = 1:n-2
%     j = i+1;
%     % Coefficients matrix
%     A(i+offset,(1:m)+m*(i-1)) = [6*T(j) 2 0 0];
%     A(i+offset,(1:m)+m*(i)) = -[6*T(j) 2 0 0];
% end
% 
% % Endpoint velocities
% A(end-1,1:m) = [3*T(1)^2 2*T(1) 1 0];
% A(end,(1:m)+m*(n-2)) = [3*T(n)^2 2*T(n) 1 0];
% 
% % Compute coeffcients
% Cx = reshape(A\bx, m, [])';
% Cy = reshape(A\by, m, [])';
% Cz = reshape(A\bz, m, [])';

%
% Evaluate polynomial
% [x,y,~]=evaluate_spline((0:0.1:max(T))',Cx,Cy,zeros(size(Cx)),T);
% t = (0:0.1:max(T))';
% 
% % AllTimes = ;
% [~, t_ind] = max(cumsum(repmat(T,size(t,1),1) <= repmat(t,1,size(T,2)),2),[],2);
% % disp(AllTimes)
% t_ind = min(t_ind,size(Cx,1));
% 
% disp(t_ind)
% % disp([t.^3 t.^2 t ones(size(t))]')
% x = diag(Cx(t_ind,:)*[t.^3 t.^2 t ones(size(t))]');
% y = diag(Cy(t_ind,:)*[t.^3 t.^2 t ones(size(t))]');
% % z = diag(Cz(t_ind,:)*[t.^3 t.^2 t ones(size(t))]');

[x,y,z] = spline.vecpos((0:0.1:max(T))');
scatter(X,Y)
hold on
plot(x,y)

