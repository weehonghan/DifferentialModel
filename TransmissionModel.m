%% Acronyms
% A: Acceleration
% Sk: Skidpad
% Sl: Slalom
% U: U-Turn

%% Inputs
% Import Engine Torque Curve
% Column A: Engine RPM
% Column B: Engine Output Torque
EngineTorqueCurve = xlsread('DiffModelEngineTorqueCurve.xlsx', 'A:B');

% Import Track Details of 4 Scenarios
accelerationDetails = xlsread('TrackDetails.xlsx', 'Acceleration');

slalomDetails = xlsread('TrackDetails.xlsx', 'Slalom');
uTurnDetails = xlsread('TrackDetails.xlsx', 'UTurn');

run TrackInitialization
run CarParametersInitialization

syms Velocity;
segmentLength = 0.1;
%% Skidpad (16.75m)
% Track Preparation
run SkidPadInitialization

% HERE
%% Data Initialization
accumulatedSegment = segmentLength;
segmentIndex = 1;

% Initialize Launch Condition
Sk_SegmentNumber(1) = 1;
Sk_Radius(1) = 999999;
Sk_MaxVelocity(1) = CarMaxVel;
Sk_CurrentVelocity(1) = LaunchVel;
Sk_Gear(1) = LaunchGear;
Sk_Acceleration(1) = 9.81; %Assume 1g accerlation **doesnt really matters
Sk_CurrentVelocity(2) = LaunchVel;

% Initialize Segment Number and Radius
for i = 2:numSegments
    Sk_SegmentNumber(i) = i;
    if (accumulatedSegment >= (skidpadDetails(segmentIndex, 2)+0.00001))
        segmentIndex = segmentIndex + 1;
        accumulatedSegment = segmentLength;
    end
    Sk_Radius(i) = skidpadDetails(segmentIndex, 3); %#ok<*SAGROW>
    accumulatedSegment = accumulatedSegment + segmentLength;
end

% L = matrix
% D = Start
% E = End

% Initialize CornerEntry Matrix
CornerEntryMat = zeros(skidpadRows, 2);
D = 2;
E = 1;
kk = 1;
for k = 1:skidpadRows
    if ( skidpadDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 1) = D;
    end
    D = D + ( skidpadDetails(k, 2) / segmentLength );
    E = E + ( skidpadDetails(k, 2) / segmentLength );
    if ( skidpadDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 2) = E;
        kk = kk + 1;
    end;
end

for iii=1:skidpadCorners
   MidCornerIndex=round((CornerEntryMat(iii,1)+ CornerEntryMat(iii,2))/2);
   CornerEntryGradient=AvaliableMovement/(MidCornerIndex-CornerEntryMat(iii,1));
   CornerExitGradient=0.15; %Need to change this crude assumption. This is assuming that the radius of turn changes linearly at corner exit.
   % The actual relationship is exponential in nature and it differs from corner to corner 
   %CornerExitGradient=AvaliableMovement/(CornerEntryMat(iii,2)-MidCornerIndex);
    
   for InnerLoop = CornerEntryMat(iii,1):  MidCornerIndex-1 
   Sk_Radius(InnerLoop+1)= Sk_Radius(InnerLoop) - CornerEntryGradient;
   end
   
   for InnerLoop2 = MidCornerIndex:CornerEntryMat(iii,2)-1
   Sk_Radius(InnerLoop2+1)= Sk_Radius(InnerLoop2) + CornerExitGradient;    
   end   
end

for j = 2:numSegments
    if (Sk_Radius(j) == 999999)
        Sk_MaxVelocity(j, 3) = CarMaxVel;           
    elseif (Sk_Radius(j) == Sk_Radius(j-1))
         Sk_MaxVelocity(j)= Sk_MaxVelocity(j-1);
    else
        Sk_MaxVelocity(j) = 3.6 * vpasolve(-TotalMass*Velocity^2/Sk_Radius(j) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,[0 150]);
       %Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,Lap(j-1, 3));
    end
end