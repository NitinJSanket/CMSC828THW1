function [AllPose, AllPoseIdeal] = MoveRobot(Pose, OdomCov, NumSteps)
%% Motion Model
% Assumed that no noise in motion model as all the motion noise is included in odometry
MotionNoise = [0, 0, 0]';

% Rotation is decoupled from movement, the robot rotates first to align
% itself and then moves to the desired point
MotionUpdate = @(Pose, Odom, OdomCov, MotionNoise)(Pose + Odom + normrnd([0,0,0]', Odom.*OdomCov) + normrnd([0,0,0]', MotionNoise));

AllPose = zeros(size(Pose,1), NumSteps+1);
AllPose(:,1) = Pose;
% Fixed motion
Odom = [2, 2, deg2rad(5)]';

for count = 1:NumSteps
    AllPose(:,count+1) = MotionUpdate(AllPose(:, count), Odom, OdomCov, MotionNoise);
end

% FOR DEBUGGING ONLY!
AllPoseIdeal(:,1) = Pose;
for count = 1:NumSteps
    AllPoseIdeal(:,count+1) = AllPose(:, count) + Odom;
end
end