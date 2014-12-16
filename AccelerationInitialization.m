details = xlsread('TrackDetails.xlsx', 'Acceleration');
track = 'Acceleration';

run DataInitialization

% Initialize Launch Condition
SegmentNumber(1) = 1;
Radius(1) = 999999;
MaxVelocity(1) = CarMaxVel;
Velocity(1) = 0;
Gear(1) = 1;
Acceleration(1) = 9.81; %Assume 1g accerlation **doesnt really matters
Velocity(2) = Velocity(1);