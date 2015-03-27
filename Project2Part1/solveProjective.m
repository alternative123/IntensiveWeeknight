function [H, A] = solveProjective(p1, p2)
% solveProjective
% Function that find the best projective transformation to map points1
% to points2
% Inputs:
%  p{1,2} - points in R^(3 x n), solving for H*p1 ~ p2, with third element
%           always being 1
% Outputs:
%  H - the 3 by 3 transformation that maps p1 to p2 (Homography matrix)
%  A - the final matrix that was SVD'd (for debugging purposes)

% get number of points
[~, npoints] = size(p1);

% Allocate the matrix size
A = zeros(2*npoints, 9);

% Instantiate the matrix
for i = 1:npoints
   row = 2*(i-1) + 1;
   % Set up the two rows associated with this point relation
   A(row, :)     = [p1(:,i)', zeros(1,3), -p2(1,i)*p1(:,i)'];
   A(row + 1, :) = [zeros(1,3), p1(:,i)', -p2(2,i)*p1(:,i)'];
end
% Find the kernel of this matrix using SVD
[~, ~, V] = svd(A);
x = V(:, 9);    % The last column of V is its kernel

% Return the H matrix
H = reshape(x,3,3)' / x(9);

end

