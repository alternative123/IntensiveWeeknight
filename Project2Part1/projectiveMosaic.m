function finalImage = projectiveMosaic(I1, I2)
    % Use sift to get the feature from the image
    [points1, points2] = findPointsSift(I1, I2);
    
    % Use RANSAC to get a good model
    nruns = 1000; % nruns is arbitrary - this numbers seems to work well
    thresh = 5; % This is also arbitrary. Just a minimum distance
    H = findHomographyWithRansac(points1, points2, nruns, thresh);
    
    % Mesh the images together into one
    finalImage = meshImages(I1, I2, H);
    %finalImage = 2;
end


function [points1, points2, npoints] = findPointsSift(I1, I2)
% findPointsSift
% Finds the points on images I1 and I2 using sift
% Warning: sift can be error prone, need to filter out more points
% Inputs:
%  I1, I2 - input images to find the points of
% Outputs:
%  points1 - points of interest on image1, in homogeneous coordinates
%  points2 - points of interest on image2, in homogeneous coordinates 
%            (each correspoints to positions in points 1). Mapped
%            like:
%            [ x1 x2 ... xN ;
%              y1 y2 ... yN ;
%              1  1  ... 1 ]
% npoints - number of points

    % Using vl_feat to find the features
    [f1, d1] = vl_sift(single(rgb2gray(I1)));
    [f2, d2] = vl_sift(single(rgb2gray(I2)));

    % Filter out points to distant
    % thresh (1.75) is arbitrary, but seems to work well
    thresh = 1.75;
    [matches, ~] = vl_ubcmatch(d1, d2, thresh);

    % Get the points in the format we want
    points1 = f1(1:2, matches(1, :));
    points1(3, :) = 1;
    points2 = f2(1:2, matches(2, :));
    points2(3, :) = 1;

    % Record the number of points
    npoints = size(points1, 2);
end

function H = findHomographyWithRansac(points1, points2, nruns, thresh)
% findHomographyWithRansac
% This finds a homograpy with points1 and points2 using RANSAC
% Inputs:
%  points1, points2 - points of interest from the image to do RANSAC on.
%                     Expected that these will have errors.
%  npoints - for convienience, the number of total points
%  nruns - the number of runs, which can be increased for more accuracy but
%          also for more time
% Outputs:
%  H - the final homography 
    % Figure out how many points we have
    npoints = size(points1, 2);
    
    % Set up what we will record for RANSAC
    approximationRanks = zeros([1 nruns]); % the 'goodness' of our guesses
    inliers = cell(nruns); % The inliers for our votes
    homographies = cell(nruns); % The transformations of our guesses
    % pointsUsed = cell(nruns); % FOR DEBUGGING
    
    for run = 1:nruns
        % Choose random points to guess our transformation from
        randomPermutation = randperm(npoints);
        selectedPoints = randomPermutation(1:4);

        % pointsUsed{run} = selectedPoints; % FOR DEBUGGING

        % Find the map of points1 to points2
        H = solveProjective( ...
                points1(:, selectedPoints), ...
                points2(:, selectedPoints));

        % See how our guess compares to the actual points
        guessed = H*points1;
        guessed = bsxfun(@rdivide, guessed, guessed(3, :));
        actual = points2;

        % Get the distances between the guessed points and actual points
        differences = guessed - actual;
        distances = dot(differences, differences);
        
        % Calculate how many are inliers and make our rank based of that
        inliers{run} = distances < thresh^2;
        rank = sum(inliers{run}); % each inlier 'votes'
        
        % Record our results
        approximationRanks(run) = rank;
        homographies{run} = H;
    end
    
    % Get the best transformation using only the inlier points
    [~, bestGuess] = max(approximationRanks);
    H = solveProjective( ...
                points1(:, inliers{bestGuess}), ...
                points2(:, inliers{bestGuess}));
    % H / H(9)
    % pointsUsedFinal = pointsUsed{bestGuess}; % FOR DEBUGGING

end

function [H, A] = solveProjective(points1, points2)
% solveProjective
% Function that find the best projective transformation to map points1
% to points2
% Inputs:
%  points1, points2 - the two sets of points to find the trasnformation for
% Outputs:
%  H - the 3 by 3 transformation that maps points1 to points2
%  A - the final matrix that was SVD'd (for debugging purposes)


    % get number of points
    [~, npoints] = size(points1);
    
    % Allocate the matrix size
    A = zeros(2*npoints, 9);

    % Instantiate the matrix
    for i = 1:npoints
       row = 2*(i-1) + 1;
       
       % Get appropriate points
       x1 = points1(1, i);
       x2 = points1(2, i);
       y1 = points2(1, i);
       y2 = points2(2, i);
       
       % Set up the two rows associated with this point relation
       A(row, :)     = [x1, x2, 1, 0,  0,  0, -x1*y1, -x2*y1, -y1];
       A(row + 1, :) = [0,  0,  0, x1, x2, 1, -x1*y2, -x2*y2, -y2];
    end
    % Find the kernel of this matrix using SVD
    [~, ~, V] = svd(A);
    x = V(:, 9);    % The last column of V is its kernel
    
    % Return the H matrix
    H = [ x(1), x(2), x(3) ;
          x(4), x(5), x(6) ;
          x(7), x(8), x(9) ];
end

function image = meshImages(I1, I2, H)
% meshImages
% Meshes images I1 and I2 together using H inverse on I2. Uses the vl_feat
% function vl_imwbackward. Based on the sift_mosaic image warping
% Inputs:
%  I1, I2 - images to be put together. I2 gets transformed, I1 stays the
%           same
%  H - the 3 by 3 projective matrix of the transformation, mapping I1 to I2
% Outputs:
%  image - the image with I1 and I2 put together
    
    % Get the corners of I2
    corners = [ 1 size(I2, 2) size(I2, 2) 1;
                1 size(I2, 1) 1           size(I2, 1) ;
                1 1           1           1 ];

    % Map the corners of I2 to their transformed locations
    cornersInv = H \ corners;
    cornersInv(1, :) = cornersInv(1, :) ./ cornersInv(3, :);
    cornersInv(2, :) = cornersInv(2, :) ./ cornersInv(3, :);
    cornersInv(3, :) = 1;

    % Get the ranges needed to map I2 onto I1's domain
    xrange = ...
        min([1, cornersInv(1,:)]) : max([size(I1,2), cornersInv(1,:)]);
    yrange = ...
        min([1, cornersInv(2,:)]) : max([size(I1,1), cornersInv(2,:)]);
    [xmesh1, ymesh1] = meshgrid(xrange, yrange); 
    % have the right meshes now - cover all the range of I1

    % Get I1 with the appropriate padding
    I1_tformed = vl_imwbackward(im2double(I1), xmesh1, ymesh1);

    % Transform the mesh (representing domain of I1)
    % into the domain of I2, to see what colors we
    % need to extract
    xmesh2 = H(1,1)*xmesh1 + H(1,2)*ymesh1 + H(1,3);
    ymesh2 = H(2,1)*xmesh1 + H(2,2)*ymesh1 + H(2,3);

    % Account for the scale factor in the projective transformation
    scaleFactor = H(3,1)*xmesh1 + H(3,2)*ymesh1 + H(3,3);
    xmesh2 = xmesh2 ./ scaleFactor;
    ymesh2 = ymesh2 ./ scaleFactor;

    % Transform I2 into I1's plane
    I2_tformed = vl_imwbackward(im2double(I2), xmesh2, ymesh2);

    % Overlap the images
    mask = (~isnan(I1_tformed) + ~isnan(I2_tformed));
    % This gets where the images overlap
    % overlapMask = (mask == 2); 
    % I1_tformed(overlapMask) = 0; % Zero out overlap area
    % Alternative to zeroing out overlap is averaging the overlap of
    % the two pictures
    
    I1_tformed(isnan(I1_tformed)) = 0;
    I2_tformed(isnan(I2_tformed)) = 0;
    
    % Combine to make the final image
    % Average where they overlap
    image = (I1_tformed + I2_tformed) ./ mask;

    corners
    cornersInv
    imshow(image)
    whyohwhy = ginput(4)
end













