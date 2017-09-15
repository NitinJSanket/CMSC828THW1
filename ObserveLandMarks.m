function ObservedLandMarks = ObserveLandMarks(Pose, CamFOV, CamCov, CamMaxDist, LandMarks, PDetLandMark, PDetLandMarkIdx)
% For a point to be inside the observable region the angle made by the
% point and the camera center should be within the observable FOV and the
% point should be within the CamMaxDist
Ang = atan2(LandMarks(:,2)-Pose(2), LandMarks(:,1)-Pose(1));
Dist = sqrt((LandMarks(:,1)-Pose(1)).^2 + (LandMarks(:,2)-Pose(2)).^2);
PointsInside = Ang<=CamFOV/2+Pose(3) & Ang>=-CamFOV/2+Pose(3) & Dist<=CamMaxDist;
NumLandMarks = size(LandMarks, 1);
% Probability of "seeing" landmarks which are in your FOV
ProbIdxSeeing = (rand(NumLandMarks, 1)<=PDetLandMark).*(PointsInside); % Mask by landmarks inside
% Probability of seeing correct Idx for landmarks you "see"
% ProbIdxSeeing = (rand(NumLandMarks, 1)<=PDetLandMarkIdx).*(ProbSeeing); % Mask by "seen" landmarks inside
% Assign Indexes too observed landmarks
% First correctly observed landmarks
CorrSeen = ProbIdxSeeing;
ObservedLandMarksIdx = find(CorrSeen)';

% WrongSeen = (ProbSeeing~=ProbIdxSeeing);
% Distance to landmark as seen by the sensor with some noise
NumObservedLandMarks = length(ObservedLandMarksIdx);
ObservedLandMarksLocs = [LandMarks(ObservedLandMarksIdx,1)-Pose(1), LandMarks(ObservedLandMarksIdx,2)-Pose(2)] +...
    normrnd(zeros(NumObservedLandMarks,2), repmat(CamCov',NumObservedLandMarks, 1));

ObservedLandMarks.Locations = ObservedLandMarksLocs;
ObservedLandMarks.Idx = ObservedLandMarksIdx;
% FOR DEBUGGING ONLY!
CorrUnseen = (ProbIdxSeeing~=PointsInside); 
ObservedLandMarksIdxNon = find(CorrUnseen)';
NumObservedLandMarksNon = length(ObservedLandMarksIdxNon);
ObservedLandMarksLocsNon = [LandMarks(ObservedLandMarksIdxNon,1)-Pose(1), LandMarks(ObservedLandMarksIdxNon,2)-Pose(2)] +...
    normrnd(zeros(NumObservedLandMarksNon,2), repmat(CamCov',NumObservedLandMarksNon, 1));

ObservedLandMarks.LocationsNon = ObservedLandMarksLocsNon;
ObservedLandMarks.IdxNon = ObservedLandMarksIdxNon;
end
