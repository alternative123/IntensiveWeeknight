function [ x, y, z ] = evaluate_spline(t,Cx,Cy,Cz,T)
% Evalute the spline determined by coefficients Cx, Cy, Cz in R^(n-1 x m)
% (m the order of the spline, n the number of points). T must be 1 x n, and
% t must be k x 1 (k being the numer of points we are evaluating)

[~, t_ind] = max(cumsum(repmat(T,size(t,1),1) <= repmat(t,1,size(T,2)),2),[],2);
t_ind = min(t_ind,size(Cx,1));

% Create the appropriate positions using the coefficients
x = sum(Cx(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);
y = sum(Cy(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);
z = sum(Cz(t_ind,:).*[t.^3 t.^2 t ones(size(t))], 2);

end