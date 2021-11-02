addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/spatial_chain/
addpath ../../../../../Simulator/yet-another-robotics-toolbox/code/

ccc
folder = 'data/multi_tracking/';

data = dlmread(strcat(folder,'Sigma_0.03_Euler.txt'))';
motor = data(1,:);
pt1 = data(2:4,:);
pt2 = data(5:7,:);
pt3 = data(8:10,:);
pt4 = data(11:13,:);


% tracker = trackerJPDA('TrackLogic','History', 'AssignmentThreshold',100,...
%     'ConfirmationThreshold', [4 5], ...
%     'DeletionThreshold', [10 10]);