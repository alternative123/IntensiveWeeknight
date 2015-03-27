% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

estimate_pose_handle = @(sensor) estimate_pose(sensor);


%%
pts = 1:12;
scatter(d.p0(1,pts),d.p0(2,pts),'.'); hold on; 
scatter(d.p1(1,pts),d.p1(2,pts)); hold on; 
scatter(d.p2(1,pts),d.p2(2,pts)); hold on; 
scatter(d.p3(1,pts),d.p3(2,pts)); hold on;
scatter(d.p4(1,pts),d.p4(2,pts)); hold on;

%%
     


