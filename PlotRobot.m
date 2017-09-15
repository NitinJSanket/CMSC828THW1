function PlotRobot(Pose, MarkerSpec)
% Plot robot
hold on;
plot(Pose(1,:), Pose(2,:), MarkerSpec, 'MarkerSize', 10);
hold off;
end