addpath(ToolboxPath);
import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - The robot and landmarks are on a grid, moving 2 meters each step
%  - Landmarks are 2 meters away from the robot trajectory

%% Create keys for variables
% Variables to store state
x = cell(NumSteps+1, 1);
for count = 1:NumSteps+1
    x{count} = symbol('x', count);
end
% Variables to store landmarks
NumLandMarksObserved = [];
for count = 1:length(ObservedLandMarks)
    NumLandMarksObserved = [NumLandMarksObserved, ObservedLandMarks{count}.Idx];
end
LandMarksObserved = unique(NumLandMarksObserved);
l = cell(length(LandMarksObserved),1);
for count = 1:length(LandMarksObserved)
    l{count} = symbol('l', LandMarksObserved(count));
end

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior to fix first pose as origin
priorMean = Pose2(0.0, 0.0, pi/2); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([1e-6; 1e-6; 1e-6]);
graph.add(PriorFactorPose2(x{1}, priorMean, priorNoise));

%% Add odometry between steps
% (odom is not fixed as [2,2,deg2rad(5)] and
% can be changed later to follow some trajectory)
% Pose2 is in units of [m,m,radians]
odometryNoise = noiseModel.Diagonal.Sigmas([0.7; 0.7; deg2rad(5)]);
for count = 1:NumSteps
    odometry = Pose2(Odom(1, count),Odom(2, count),Odom(3, count));
    graph.add(BetweenFactorPose2(x{count}, x{count+1}, odometry, odometryNoise));
end


%% Add bearing/range measurement factors
brNoise = noiseModel.Diagonal.Sigmas([0.2; deg2rad(2)]);
for step = 1:NumSteps+1
    for count = 1:length(ObservedLandMarks{step}.Idx)
        LandMarkIdx = find(LandMarksObserved == ObservedLandMarks{step}.Idx(count));
        graph.add(BearingRangeFactor2D(x{step}, l{LandMarkIdx},...
        Rot2(BearingMeasurements{step}.Angle(count)), BearingMeasurements{step}.Distance(count), brNoise));
    end
end

% print
graph.print(sprintf('\nFull graph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(x{1}, Pose2(0.0 ,0.0 ,0.0));
for count = 1:NumSteps
    initialEstimate.insert(x{count+1}, Pose2(rand , rand, rand));
end
for count = 1:length(LandMarksObserved)
    initialEstimate.insert(l{count}, Point2(2.*WorldLim.*rand - WorldLim,...
        2.*WorldLim.*rand - WorldLim));
end

initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
result = optimizer.optimizeSafely();
result.print(sprintf('\nFinal result:\n'));


% params = DoglegParams;
% params.setAbsoluteErrorTol(1e-15);
% params.setRelativeErrorTol(1e-15);
% params.setVerbosity('ERROR');
% params.setVerbosityDL('VERBOSE');
% params.setOrdering(graph.orderingCOLAMD());
% optimizer = DoglegOptimizer(graph, initialEstimate, params);                      
% 
% result = optimizer.optimizeSafely();
% result.print('final result');

%% Plot Covariance Ellipses
cla;hold on

marginals = Marginals(graph, result);
plot2DTrajectory(result, [], marginals);
plot2DPoints(result, 'b', marginals);

% for count = 1:NumSteps+1
% plot([result.at(x{count}).x; result.at(j1).x],[result.at(i1).y; result.at(j1).y], 'c-');
% plot([result.at(i2).x; result.at(j1).x],[result.at(i2).y; result.at(j1).y], 'c-');
% plot([result.at(i3).x; result.at(j2).x],[result.at(i3).y; result.at(j2).y], 'c-');
% axis([-0.6 4.8 -1 1])
% axis equal
% view(2)

