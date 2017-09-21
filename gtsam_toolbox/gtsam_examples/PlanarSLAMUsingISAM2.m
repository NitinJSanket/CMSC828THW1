%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation,
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
%
% See LICENSE for the license information
%
% @brief Simple robotics example using the pre-built planar SLAM domain
% @author Alex Cunningham
% @author Frank Dellaert
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all

import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Initialize iSAM
params = gtsam.ISAM2Params;
params.setOptimizationParams(gtsam.ISAM2DoglegParams)
isam = ISAM2(params);


%% Create keys for variables
i1 = symbol('x',1);
j1 = symbol('l',1);

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior
priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([0.001; 0.001; 0.001]);
graph.add(PriorFactorPose2(i1, priorMean, priorNoise));

%% Add bearing/range measurement factors
degrees = pi/180;
brNoise = noiseModel.Diagonal.Sigmas([0.1; 0.2]);
graph.add(BearingRangeFactor2D(i1, j1, Rot2(45*degrees), sqrt(8), brNoise));

% print
graph.print(sprintf('\nFull graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(i1, Pose2(0.5, 0.0, 0.2));
initialEstimate.insert(j1, Point2(1.8, 2.1));
initialEstimate.print(sprintf('\nInitial estimate:\n'));


%% Add odometry between i1 and i2 and observation between i2 and l1
% iSAM expects us to give it a new set of factors
% along with initial estimates for any new variables introduced.
odometry = Pose2(2.0, 0.0, 0.0);
odometryNoise = noiseModel.Diagonal.Sigmas([0.2; 0.2; 0.1]);
i2 = symbol('x',2);
graph.add(BetweenFactorPose2(i1, i2, odometry, odometryNoise));
graph.add(BearingRangeFactor2D(i2, j1, Rot2(90*degrees), 2, brNoise));
initialEstimate.insert(i2, Pose2(2.3, 0.1,-0.2));


% batchOptimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
% fullyOptimized = batchOptimizer.optimize();
% isam.update(graph, fullyOptimized);
isam.update(graph, initialEstimate);
result1 = isam.calculateEstimate();


%%  Add odometry between i2 and i3 and observation between i3 and l2
newFactors = NonlinearFactorGraph;
initialEstimates = Values;
i3 = symbol('x',3);
j2 = symbol('l',2);
priorNoise2 = noiseModel.Diagonal.Sigmas([0.5; 0.5; 0.25]);
newFactors.add(PriorFactorPose2(i3, result1.at(i2), priorNoise2));
newFactors.add(BetweenFactorPose2(i2, i3, odometry, odometryNoise));
newFactors.add(BearingRangeFactor2D(i3, j2, Rot2(90*degrees), 2, brNoise));
initialEstimates.insert(i3, Pose2(4.1, 0.1, 0.1));
initialEstimates.insert(j2, Point2(4.1, 1.8));
isam.update(newFactors, initialEstimates);
result = isam.calculateEstimate();
