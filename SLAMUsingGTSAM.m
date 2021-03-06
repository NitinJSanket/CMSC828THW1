addpath(ToolboxPath);
import gtsam.*

%% Assumptions
%  - All values are axis aligned
%  - Robot poses are facing along the X axis (horizontal, to the right in images)
%  - We have bearing and range information for measurements
%  - We have full odometry for measurements
%  - In each step, the robot moves (2,2) in x and y.
%  - Landmarks visible to the robot are in a 15 meter radius from the robot

%% Create keys for variables
% Variables to store state
x = cell(NumSteps+1, 1);
for count = 1:NumSteps+1
    x{count} = symbol('x', count);
end
% Variables to store landmarks
LandMarksObserved = [];
for count = 1:length(ObservedLandMarks)
    LandMarksObserved = [LandMarksObserved, ObservedLandMarks{count}.Idx];
end
LandMarksObserved = unique(LandMarksObserved);
l = cell(length(LandMarksObserved),1);
for count = 1:length(LandMarksObserved)
    l{count} = symbol('l', LandMarksObserved(count));
end

%% Create graph container and add factors to it
graph = NonlinearFactorGraph;

%% Add prior to fix first pose as origin
priorMean = Pose2(-10.0, -10.0, pi/2); % prior at origin
priorNoise = noiseModel.Diagonal.Sigmas([1e-6; 1e-6; 1e-6]);
graph.add(PriorFactorPose2(x{1}, priorMean, priorNoise));

%% Add odometry between steps
% (odom is not fixed as [2,2,deg2rad(5)] and
% can be changed later to follow some trajectory)
% Pose2 is in units of [m,m,radians]
odometryNoise = noiseModel.Diagonal.Sigmas([1.0; 1.0; deg2rad(10)]);
for count = 1:NumSteps
    odometry = Pose2(Odom(1, count),Odom(2, count),Odom(3, count));
    graph.add(BetweenFactorPose2(x{count}, x{count+1}, odometry, odometryNoise));
end


%% Add bearing/range measurement factors
brNoise = noiseModel.Diagonal.Sigmas([0.2; deg2rad(10)]);
for step = 1:NumSteps+1
    for count = 1:length(ObservedLandMarks{step}.Idx)
        LandMarkIdx = find(LandMarksObserved == ObservedLandMarks{step}.Idx(count));
        graph.add(BearingRangeFactor2D(x{step}, l{LandMarkIdx},...
            Rot2(BearingMeasurements{step}.Angle(count)), BearingMeasurements{step}.Distance(count), brNoise));
    end
end

% print
graph.print(sprintf('\nFull grObservedLandMarks{step}.Idxaph:\n'));

%% Initialize to noisy points
initialEstimate = Values;
initialEstimate.insert(x{1}, Pose2(-10.0 ,-10.0 , pi/2));
for count = 2:NumSteps+1
    initialEstimate.insert(x{count}, Pose2(rand , rand, rand));
end
for count = 1:length(LandMarksObserved)
    %     initialEstimate.insert(l{count}, Point2(2.*WorldLim.*rand - WorldLim,...
    %         2.*WorldLim.*rand - WorldLim));
    Err = 10;
    initialEstimate.insert(l{count}, Point2(LandMarks(LandMarksObserved(count),1)+rand*Err-Err/2,...
        LandMarks(LandMarksObserved(count),2)+rand*Err-Err/2));
end

initialEstimate.print(sprintf('\nInitial estimate:\n'));

%% Optimize using Levenberg-Marquardt optimization with an ordering from colamd
% optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
% result = optimizer.optimizeSafely();
% result.print(sprintf('\nFinal result:\n'));


params = DoglegParams;
params.setAbsoluteErrorTol(1e-15);
params.setRelativeErrorTol(1e-15);
params.setVerbosity('ERROR');
params.setVerbosityDL('VERBOSE');
params.setOrdering(graph.orderingCOLAMD());
optimizer = DoglegOptimizer(graph, initialEstimate, params);

result = optimizer.optimizeSafely();
result.print('final result');

% optimizer = GaussNewtonOptimizer(graph, initialEstimate);
% result = optimizer.optimizeSafely();
% result.print(sprintf('\nFinal result:\n'));
%% Fetch Results
% Each column is [x, y, theta]'
Optimizedx = zeros(3, NumSteps+1);
% Each column is [x, y, ID]'
Optimizedl = zeros(3, length(LandMarksObserved));

for step = 1:NumSteps+1
    Optimizedx(:, step) = [result.at(x{step}).x, result.at(x{step}).y, result.at(x{step}).theta]';
end


for count = 1:length(LandMarksObserved)
    Optimizedl(:, count) = [result.at(l{count}).x, result.at(l{count}).y, LandMarksObserved(count)]';
end

%% Plot Covariance Ellipses
figure,
hold on

for step = 1:NumSteps+1
    for count = 1:length(ObservedLandMarks{step}.Idx)
        LandMarkIdx = find(LandMarksObserved == ObservedLandMarks{step}.Idx(count));
        plot([result.at(x{step}).x; result.at(l{LandMarkIdx}).x],...
            [result.at(x{step}).y; result.at(l{LandMarkIdx}).y], 'c-');
    end
end

marginals = Marginals(graph, result);
plot2DTrajectory(result, [], marginals);
plot2DPoints(result, 'b', marginals);
axis equal

%% Compute Error with respect to idea values
figure,
plot(AllPoseIdeal(1,:), AllPoseIdeal(2,:), 'b.');
hold on;
plot(Optimizedx(1,:), Optimizedx(2,:), 'r.');
plot(AllPose(1,:), AllPose(2, :), 'g.');

%% Calculate Position and Angle Error
PosErr  = sqrt(sum((AllPose(1, :) -  AllPoseIdeal(1,:)).^2 + (AllPose(2, :) -  AllPoseIdeal(2,:)).^2));
AngErr = sum(AllPose(3, :) -  AllPoseIdeal(3,:));
disp(['Deadreaconing Position error per step is ', num2str(PosErr/(NumSteps+1)), ' in m']);
disp(['Deadreaconing Angle error per step is ', num2str(rad2deg(AngErr/(NumSteps+1))), ' in degrees']);
% TODO: Measure LandMark Error as well
PosErr  = sqrt(sum((Optimizedx(1, :) -  AllPoseIdeal(1,:)).^2 + (Optimizedx(2, :) -  AllPoseIdeal(2,:)).^2));
AngErr = sum(Optimizedx(3, :) -  AllPoseIdeal(3,:));
disp(['Optimized Position error per step is ', num2str(PosErr/(NumSteps+1)), ' in m']);
disp(['Optimized Angle error per step is ', num2str(rad2deg(AngErr/(NumSteps+1))), ' in degrees']);
