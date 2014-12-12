details = xlsread('TrackDetails.xlsx', 'Acceleration');
track = 'Acceleration';

rows = size(details, 1);
corners = rows - sum(details(:,1));

numSections = sum(details(:,2));
numSegments = (numSections / segmentLength) + 1 ;

SegmentNumber = zeros(numSegments, 1);
Radius = zeros(numSegments, 1);
MaxVelocity = zeros(numSegments, 1);
Velocity = zeros(numSegments, 1);
Gear = zeros(numSegments, 1);
Acceleration = zeros(numSegments, 1);
Torque = zeros(numSegments, 1);
TractiveForce = zeros(numSegments, 1);
ShiftTimeRemain = zeros(numSegments, 1);
SegmentTime = zeros(numSegments, 1);
AccumulatedTime = zeros(numSegments, 1);
Distance = zeros(numSegments, 1);

accumulatedSegment = segmentLength;
segmentIndex = 1;

% Initialize Launch Condition
SegmentNumber(1) = 1;
Radius(1) = 999999;
MaxVelocity(1) = CarMaxVel;
Velocity(1) = 0;
Gear(1) = 1;
Acceleration(1) = 9.81; %Assume 1g accerlation **doesnt really matters
Velocity(2) = Velocity(1);