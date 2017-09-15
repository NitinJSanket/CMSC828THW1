function PlotWorld(WorldLim, LandMarks)
NumLandMarks = size(LandMarks,1);
hold on;
% Plot world
axis([-WorldLim, WorldLim, -WorldLim, WorldLim].*1.05);
grid on;
% Plot landmarks
plot(LandMarks(:,1), LandMarks(:,2), 'rx');
hold off;
% Label LandMarks for easy debugging
text(LandMarks(:,1), LandMarks(:,2), num2cell(1:NumLandMarks));
axis equal
hold off;
end