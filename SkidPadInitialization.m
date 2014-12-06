skidpadDetails = xlsread('TrackDetails.xlsx', 'Skidpad');

skidpadRows = size(skidpadDetails, 1);
skidpadCorners = skidpadRows - sum(skidpadDetails(:,1));

numSections = sum(skidpadDetails(:,2));
numSegments = (numSections / segmentLength) + 1 ; 

Sk_SegmentNumber = zeros(numSegments, 1);
Sk_Radius = zeros(numSegments, 1);
Sk_MaxVelocity = zeros(numSegments, 1);
Sk_CurrentVelocity = zeros(numSegments, 1);
Sk_Gear = zeros(numSegments, 1);
Sk_Acceleration = zeros(numSegments, 1);
Sk_EngineTorque = zeros(numSegments, 1);
Sk_TractiveForce = zeros(numSegments, 1);
Sk_ShiftTimeRemain = zeros(numSegments, 1);
Sk_SegmentTime = zeros(numSegments, 1);
Sk_AccumulatedTime = zeros(numSegments, 1);
Sk_Distance = zeros(numSegments, 1);