clc 
close all
clear all
%% Simulation setup
% Sample time and time array
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:6;        % Time array

% Initial conditions
initPose = [2;2;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;

% Create visualizer
viz1 = VO;

%% Simulation Loop
rate_control = 1/sampleTime;
r = rateControl(5);

for idx = 2:numel(tVec) 
    curPose = pose(:,idx-1);
    [xp,yp,theta] = viz1(curPose,[0.7, 0.7], 2); 
    pose(:,idx) = [xp(2);yp(2);theta];
    waitfor(r);
end
