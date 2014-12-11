% Initialize Launch Condition
SegmentNumber(1) = 1;
Radius(1) = 999999;
MaxVelocity(1) = CarMaxVel;
Velocity(1) = LaunchVel;
Gear(1) = LaunchGear;
Acceleration(1) = 9.81; %Assume 1g accerlation **doesnt really matters
Velocity(2) = LaunchVel;

details = xlsread('TrackDetails.xlsx', 'Slalom');
track = 'Slalom';