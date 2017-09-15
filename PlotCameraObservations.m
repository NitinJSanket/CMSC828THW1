function PlotCameraObservations(Pose, ObservedLandMarks)
% Plot Observed LandMarks in blue
hold on;
for count = 1:size(ObservedLandMarks.Locations, 1)
    % Plot camera angle curve
    plot([Pose(1), Pose(1) + ObservedLandMarks.Locations(count,1)],...
        [Pose(2), Pose(2) + ObservedLandMarks.Locations(count,2)], 'b--');
    plot(Pose(1) + ObservedLandMarks.Locations(count,1), Pose(2) + ObservedLandMarks.Locations(count,2), 'bo');
end
hold off;

% Plot Missed LandMarks in Red
if(isfield(ObservedLandMarks, 'LocationsNon'))
    if(~isempty(ObservedLandMarks.LocationsNon))
        hold on;
        for count = 1:size(ObservedLandMarks.LocationsNon, 1)
            % Plot camera angle curve
            plot([Pose(1) Pose(1) + ObservedLandMarks.LocationsNon(count,1)],...
                [Pose(2), Pose(2) + ObservedLandMarks.LocationsNon(count,2)], 'r--');
            plot(Pose(1) + ObservedLandMarks.LocationsNon(count,1), Pose(2) + ObservedLandMarks.LocationsNon(count,2), 'ro');
        end
        hold off;
    end
end
end