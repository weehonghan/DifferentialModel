details = xlsread('TrackDetails.xlsx', 'Oval');
track = 'Oval';

run DataInitialization

% Initialize Launch Condition
SegmentNumber(1) = 1;
Radius(1) = 999999;
Direction(1) = 2;
MaxVelocity(1) = CarMaxVel;
Velocity(1) = 30;
Gear(1) = 2;
Acceleration(1) = 9.81; %Assume 1g accerlation **doesnt really matters
Velocity(2) = Velocity(1);

run MainSimulation