clc
clear all
close all

%% ASSUMPTIONS
% Obstacles magically don't block view

%% Setup World Parameters
% World Limits in each direction in m
% World size is [-WorldLim, WorldLim] in both x and y
WorldLim = 20;
% Number of Landmarks
% LandMarks are uniquely numbered between 1 to NumLandMarks
NumLandMarks = 100;
% Generate LandMarks in the World
LandMarks = 2.*WorldLim.*rand(NumLandMarks,2) - WorldLim;

%% Setup Robot Parameters
% Assume point robot
% Robot Pose is [x, y, theta]'
Pose = [-WorldLim/2, -WorldLim/2, pi/2]';

%% Setup Camera Parameters
% Camera Field of View in radians
CamFOV = deg2rad(120);
% Max. distance seen by camera sensor in m
CamMaxDist = 15;
% Camera Probability of seeing landmark (reliability of detection)
PDetLandMark = 0.95;
% Camera Probability of getting right index
PDetLandMarkIdx = 1; % UNUSED!
% Camera Distance estimator error covariance, if the camera sees the
% landmark it can estimate the distance to the landmark with this much
% error (x, y, theta) cov, assumed diagonal covariance
CamCovX = 0.1; % in m
CamCovY = 0.1; % in m
CamCov = [CamCovX, CamCovY]';

% TODO: Implement probability for landmark being given wrong index?

%% Setup Odometry Parameters
% Odometry noise covariance (x, y, theta), assumed diagonal covariance

% Odom gives change in value

OdomCovX = 0.3; % in ratio of distance moved in the range (0, 1)
OdomCovY = 0.3; % in ratio of distance moved in the range (0, 1)
OdomCovTheta = 0.05; % in ratio of angle rotated in the range (0, 1)
OdomCov = [OdomCovX, OdomCovY, OdomCovTheta]';

NumSteps = 5;
[AllPose, AllPoseIdeal] = MoveRobot(Pose, OdomCov, NumSteps);

%% Plot Robot and World 
% Plot World and Landmarks
figure,
PlotWorld(WorldLim, LandMarks);

% Plot all Robot Poses
PlotRobot(AllPose, 'b*');

% Plot all Ideal Robot Poses
PlotRobot(AllPoseIdeal, 'bo');

% Plot camera beam
PlotCamera(AllPose, CamFOV, CamMaxDist);

% Get Sensor Measurements
for count = 1:NumSteps+1
ObservedLandMarks = ObserveLandMarks(AllPose(:,count), CamFOV, CamCov, CamMaxDist, LandMarks, PDetLandMark, PDetLandMarkIdx);

% Plot camera recieving measurements with green and red arrows (for
% seeing and not seeing)
PlotCameraObservations(AllPose(:, count), ObservedLandMarks);
end

%% SLAM using Dead-reckoning
% Get Path using dead-reckoning
% Assume initial pose as (0,0,0)
PathDeadReck = [0,0,0]';
% FOR DEBUGGING ONLY
PathDeadReckIdeal = [0,0,0]';
for count = 2:NumSteps+1
    % Estimate Odom
    Odom = AllPose(:,count)-AllPose(:,count-1);
    % Add Estimated Odom to estimate pose
    PathDeadReck(:,count) = PathDeadReck(:,count-1) + Odom;
    % FOR DEBUGGING ONLY
    OdomIdeal = AllPoseIdeal(:,count)-AllPoseIdeal(:,count-1);
    PathDeadReckIdeal(:, count) = PathDeadReckIdeal(:, count-1) + OdomIdeal;
end

% Plot Estimated Path
figure,
PlotRobot(PathDeadReck, 'b*');
PlotRobot(PathDeadReckIdeal, 'bo');

%% SLAM using GTSAM





