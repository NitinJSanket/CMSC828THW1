function PlotCamera(Pose, CamFOV, CamMaxDist)
for count = 1:size(Pose, 2)
% Plot camera angle curve
hold on;
CamTheta = linspace(Pose(3, count)-CamFOV/2, Pose(3, count)+CamFOV/2, 100); % 100 points
CamX = [Pose(1, count), Pose(1, count) + CamMaxDist*cos(CamTheta)];
CamY = [Pose(2, count), Pose(2, count) + CamMaxDist*sin(CamTheta)];
patch(CamX, CamY, 'g', 'EdgeColor', 'none', 'FaceAlpha', 0.25);
hold off;
end
end