% This is a Transmission Differential Model written for NUS Formula SAE Team. 
% 
% Currently, the model assumes the following:
% - Ideal Torque Transfer(No losses in driveline) 
% - Rear tires zero camber, zero toe
%
% Author: WEE HONGHAN
% NUS Formula SAE
% Electricals/Electronics (Transmission)
% Engineering Design & Innovation Centre
% Faculty of Engineering
% 
% B.Eng. (Computer Engineering) Year 4
% Faculty of Engineering, School of Computing
% National University of Singapore
%% Current: Version 0.2
% Added Belleville Spring Calculations: InnerDiameter, OuterDiameter, YoungModulus, PoissonRatio, DiscHeight, MaterialThickness, SpringDeflection,
%                                       InsideHeight, DiameterRatio
%% Version 0.1
% Added Inputs: PowerAngle, CoastingAngle, PreloadTorque, InputTorque, EngineTorqueCurve, FrontSprocketTeeth, RearSprocketTeeth
% Added Constants: RearTrackWidth, TireRadius, EngineTorqueCurveRow, EngineTorqueCurveColumn, PrimaryRatio, Gear1Ratio, Gear2Ratio, Gear3Ratio,
%                  Gear4Ratio, Gear5Ratio, Gear6Ratio
% Added Scenarios: Acceleration, Skidpad, Slalom, Cornering U-turn
% Added Calculations: FDR, FinalDriveTorque, ForcePressureRing, SpringForce

%% Inputs
PowerAngle = 45;        % Unit = Degrees
CoastingAngle = 60;     % Unit = Degrees
PreloadTorque = 15;     % Unit = Nm
InputTorque = 38;       % Unit = Nm   ***FUNCTION OF RPM, NOT A CONSTANT***
FrontSprocketTeeth = 11;
RearSprocketTeeth = 40;
% DrivelineEfficiency = 0.94

% Import Engine Torque Curve
EngineTorqueCurve = xlsread('DiffModelEngineTorqueCurve.xlsx', 'A:B'); % Column A: Engine RPM, Column B: Engine Output Torque

% Import Track Details of 4 Scenarios
AccelerationDetails = xlsread('TrackDetails.xlsx', 'Acceleration');
SkidpadDetails = xlsread('TrackDetails.xlsx', 'Skidpad');
SlalomDetails = xlsread('TrackDetails.xlsx', 'Slalom');
UTurnDetails = xlsread('TrackDetails.xlsx', 'UTurn');

SegmentLength = 0.1;
NumVariable = 12;
CarMaxVel = 0;
LaunchVel = 50;
LaunchGear = 2;
%% Constants
CarWeight = 285;            % Unit = kg
CGHeight = 31;              % Unit = mm
%Wheelbase = 1210;           % Unit = mm
RearTrackWidth = 1190;      % Unit = mm
%b = 400;                    % Unit = mm
%c = 595;                    % Unit = mm
TireRadius = 254;           % Unit = mm
PrimaryRatio = 2.11;
Gear1Ratio = 2.750;
Gear2Ratio = 2.000;
Gear3Ratio = 1.667;
Gear4Ratio = 1.444;
Gear5Ratio = 1.304;
Gear6Ratio = 1.208;
PressureRingRadius = 0.037594;  % Unit = m
[EngineTorqueCurveRow, EngineTorqueCurveColumn] = size(EngineTorqueCurve);
EngineTorqueCurve = round(EngineTorqueCurve);

Gravity = 9.81;             % Unit = m/s^2
%a = Wheelbase - b;          % Unit = mm
%d = RearTrackWidth - c;     % Unit = mm
CoFRoad = 0.7;

% RearSlipAngle?
% LatForce

%% Scenarios
%% Acceleration (75m)
% - 100 Data points
% Track Preparation 
AccelSize = size(AccelerationDetails);
S = AccelSize(1,1);

LastSegment = sum(AccelerationDetails(:,2));
% second part
LastSegment = (LastSegment / SegmentLength) + 1 ;
Lap = zeros(LastSegment , NumVariable);
tC = SegmentLength;
ii = 1;
Lap(1, 1) = 1;
    
for j = 2:LastSegment
    Lap(j, 1) = j;
    if (tC >= (AccelerationDetails(ii, 2)+0.00001))
        ii = ii + 1;
        tC = SegmentLength;
    end
    Lap(j, 2) = AccelerationDetails(ii, 3);
    tC = tC + SegmentLength;
end

% third part
CornerEntryMat = zeros(S, 2);
D = 2;
E = 1;
kk = 1;
for k = 1:S
    if ( AccelerationDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 1) = D;
    end
    D = D + ( AccelerationDetails(k, 2) / SegmentLength );
    E = E + ( AccelerationDetails(k, 2) / SegmentLength );
    if ( AccelerationDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 2) = E;
        kk = kk + 1;
    end;
end

for j = 2:LastSegment
    if (Lap(j, 2) == 999999)
        Lap(j, 3) = CarMaxVel;           
    elseif (Lap(j, 2) == Lap(j-1, 2))
         Lap(j, 3)= Lap(j-1, 3);
    else
        Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,[0 150]);
       %Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,Lap(j-1, 3));
    end
end
% L = matrix
% D = Start
% E = End

%%Launch Condition (INPUT)
Lap(1,2)=999999;
Lap(1,3)=CarMaxVel;
Lap(1,4)=LaunchVel;
Lap(1,5)=LaunchGear;
Lap(1,6)=9.81; %Assume 1g accerlation **doesnt really matters
Lap(2,4)=LaunchVel;

%% Skidpad (16.75m)
% - 100 Data points (25-25-25-25)
% Track Preparation 
SkidpadSize = size(SkidpadDetails);
SkidpadRows = SkidpadSize(1,1);
NumCorner = SkidpadRows - sum(SkidpadDetails(:,1));

LastSegment = sum(SkidpadDetails(:,2));
% second part
LastSegment = (LastSegment / SegmentLength) + 1 ;
Lap = zeros(LastSegment , NumVariable);
tC = SegmentLength;
ii = 1;
Lap(1, 1) = 1;
    
for j = 2:LastSegment
    Lap(j, 1) = j;
    if (tC >= (SkidpadDetails(ii, 2)+0.00001))
        ii = ii + 1;
        tC = SegmentLength;
    end
    Lap(j, 2) = SkidpadDetails(ii, 3);
    tC = tC + SegmentLength;
end

% third part
CornerEntryMat = zeros(S, 2);
D = 2;
E = 1;
kk = 1;
for k = 1:S
    if ( SkidpadDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 1) = D;
    end
    D = D + ( SkidpadDetails(k, 2) / SegmentLength );
    E = E + ( SkidpadDetails(k, 2) / SegmentLength );
    if ( SkidpadDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 2) = E;
        kk = kk + 1;
    end;
end

for iii=1:NumCorner
   MidCornerIndex=round((CornerEntryMat(iii,1)+ CornerEntryMat(iii,2))/2);
   CornerEntryGradient=AvaliableMovement/(MidCornerIndex-CornerEntryMat(iii,1));
   CornerExitGradient=0.15; %Need to change this crude assumption. This is assuming that the radius of turn changes linearly at corner exit.
   % The actual relationship is exponential in nature and it differs from corner to corner 
   %CornerExitGradient=AvaliableMovement/(CornerEntryMat(iii,2)-MidCornerIndex);
    
   for InnerLoop = CornerEntryMat(iii,1):  MidCornerIndex-1 
   Lap(InnerLoop+1,2)= Lap(InnerLoop,2) - CornerEntryGradient;
   end
   
   for InnerLoop2 = MidCornerIndex:CornerEntryMat(iii,2)-1
   Lap(InnerLoop2+1,2)= Lap(InnerLoop2,2) + CornerExitGradient;    
   end   
end

for j = 2:LastSegment
    if (Lap(j, 2) == 999999)
        Lap(j, 3) = CarMaxVel;           
    elseif (Lap(j, 2) == Lap(j-1, 2))
         Lap(j, 3)= Lap(j-1, 3);
    else
        Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,[0 150]);
       %Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,Lap(j-1, 3));
    end
end
% L = matrix
% D = Start
% E = End

% Launch Condition (INPUT)
Lap(1,2)=999999;
Lap(1,3)=CarMaxVel;
Lap(1,4)=LaunchVel;
Lap(1,5)=LaunchGear;
Lap(1,6)=9.81; %Assume 1g accerlation **doesnt really matters
Lap(2,4)=LaunchVel;

%% Slalom (10m spacing)
% - 100 Data points (25-25-25-25)
% Track Preparation 
Size1=size(SlalomDetails);
S=Size1(1,1);
NumCorner=S-sum(SlalomDetails(:,1));

LastSegment = sum(SlalomDetails(:,2));
% second part
LastSegment = (LastSegment / SegmentLength) + 1 ;
Lap = zeros(LastSegment , NumVariable);
tC = SegmentLength;
ii = 1;
Lap(1, 1) = 1;
    
for j = 2:LastSegment
    Lap(j, 1) = j;
    if (tC >= (SlalomDetails(ii, 2)+0.00001))
        ii = ii + 1;
        tC = SegmentLength;
    end
    Lap(j, 2) = SlalomDetails(ii, 3);
    tC = tC + SegmentLength;
end

% third part
CornerEntryMat = zeros(S, 2);
D = 2;
E = 1;
kk = 1;
for k = 1:S
    if ( SlalomDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 1) = D;
    end
    D = D + ( SlalomDetails(k, 2) / SegmentLength );
    E = E + ( SlalomDetails(k, 2) / SegmentLength );
    if ( SlalomDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 2) = E;
        kk = kk + 1;
    end;
end

for iii=1:NumCorner
   MidCornerIndex=round((CornerEntryMat(iii,1)+ CornerEntryMat(iii,2))/2);
   CornerEntryGradient=AvaliableMovement/(MidCornerIndex-CornerEntryMat(iii,1));
   CornerExitGradient=0.15; %Need to change this crude assumption. This is assuming that the radius of turn changes linearly at corner exit.
   % The actual relationship is exponential in nature and it differs from corner to corner 
   %CornerExitGradient=AvaliableMovement/(CornerEntryMat(iii,2)-MidCornerIndex);
    
   for InnerLoop = CornerEntryMat(iii,1):  MidCornerIndex-1 
   Lap(InnerLoop+1,2)= Lap(InnerLoop,2) - CornerEntryGradient;
   end
   
   for InnerLoop2 = MidCornerIndex:CornerEntryMat(iii,2)-1
   Lap(InnerLoop2+1,2)= Lap(InnerLoop2,2) + CornerExitGradient;    
   end   
end

for j = 2:LastSegment
    if (Lap(j, 2) == 999999)
        Lap(j, 3) = CarMaxVel;           
    elseif (Lap(j, 2) == Lap(j-1, 2))
         Lap(j, 3)= Lap(j-1, 3);
    else
        Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,[0 150]);
       %Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,Lap(j-1, 3));
    end
end
% L = matrix
% D = Start
% E = End

%%Launch Condition (INPUT)
Lap(1,2)=999999;
Lap(1,3)=CarMaxVel;
Lap(1,4)=LaunchVel;
Lap(1,5)=LaunchGear;
Lap(1,6)=9.81; %Assume 1g accerlation **doesnt really matters
Lap(2,4)=LaunchVel;

%% U-Turn Cornering (7.25m)
% - 100 Data points (25-25-25-25)
% Track Preparation 
Size1=size(UTurnDetails);
S=Size1(1,1);
NumCorner=S-sum(UTurnDetails(:,1));

LastSegment = sum(UTurnDetails(:,2));
% second part
LastSegment = (LastSegment / SegmentLength) + 1 ;
Lap = zeros(LastSegment , NumVariable);
tC = SegmentLength;
ii = 1;
Lap(1, 1) = 1;
    
for j = 2:LastSegment
    Lap(j, 1) = j;
    if (tC >= (UTurnDetails(ii, 2)+0.00001))
        ii = ii + 1;
        tC = SegmentLength;
    end
    Lap(j, 2) = UTurnDetails(ii, 3);
    tC = tC + SegmentLength;
end

% third part
CornerEntryMat = zeros(S, 2);
D = 2;
E = 1;
kk = 1;
for k = 1:S
    if ( UTurnDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 1) = D;
    end
    D = D + ( UTurnDetails(k, 2) / SegmentLength );
    E = E + ( UTurnDetails(k, 2) / SegmentLength );
    if ( UTurnDetails(k, 3) ~= 999999 )
        CornerEntryMat(kk, 2) = E;
        kk = kk + 1;
    end;
end

for iii=1:NumCorner
   MidCornerIndex=round((CornerEntryMat(iii,1)+ CornerEntryMat(iii,2))/2);
   CornerEntryGradient=AvaliableMovement/(MidCornerIndex-CornerEntryMat(iii,1));
   CornerExitGradient=0.15; %Need to change this crude assumption. This is assuming that the radius of turn changes linearly at corner exit.
   % The actual relationship is exponential in nature and it differs from corner to corner 
   %CornerExitGradient=AvaliableMovement/(CornerEntryMat(iii,2)-MidCornerIndex);
    
   for InnerLoop = CornerEntryMat(iii,1):  MidCornerIndex-1 
   Lap(InnerLoop+1,2)= Lap(InnerLoop,2) - CornerEntryGradient;
   end
   
   for InnerLoop2 = MidCornerIndex:CornerEntryMat(iii,2)-1
   Lap(InnerLoop2+1,2)= Lap(InnerLoop2,2) + CornerExitGradient;    
   end   
end

for j = 2:LastSegment
    if (Lap(j, 2) == 999999)
        Lap(j, 3) = CarMaxVel;           
    elseif (Lap(j, 2) == Lap(j-1, 2))
         Lap(j, 3)= Lap(j-1, 3);
    else
        Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,[0 150]);
       %Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,Lap(j-1, 3));
    end
end
% L = matrix
% D = Start
% E = End

% Launch Condition (INPUT)
Lap(1,2)=999999;
Lap(1,3)=CarMaxVel;
Lap(1,4)=LaunchVel;
Lap(1,5)=LaunchGear;
Lap(1,6)=9.81; %Assume 1g accerlation **doesnt really matters
Lap(2,4)=LaunchVel;
%% Calculations
FDR = RearSprocketTeeth/FrontSprocketTeeth;
FinalDriveTorque = InputTorque*PrimaryRatio*Gear1Ratio*FDR;                 % Unit = Nm
ForcePressureRing = FinalDriveTorque/PressureRingRadius;                    % Unit = N
LateralForcePressureRing = tand(90-PowerAngle)*ForcePressureRing/2;         % Unit = N
%% Belleville Spring Calculations
% http://www.mitcalc.com/doc/springs/help/en/springs.htm
InnerDiameter = 46;         % Unit = mm
OuterDiameter = 75.8;       % Unit = mm
YoungModulus = 207;         % Unit = GPa
PoissonRatio = 0.3;     
DiscHeight = 3.3;           % Unit = mm
MaterialThickness = 1.6;    % Unit = mm
SpringDeflection = 1.7;     % Unit = mm
InsideHeight = DiscHeight-MaterialThickness;        % Unit = mm
DiameterRatio = OuterDiameter/InnerDiameter;
ShapeCoeff1 = (1/pi)*(((DiameterRatio-1)/DiameterRatio)^2/(((DiameterRatio+1)/(DiameterRatio-1))-(2/log(DiameterRatio))));
ShapeCoeff2 = (6/pi)*((((DiameterRatio-1)/log(DiameterRatio))-1)/(log(DiameterRatio)));
ShapeCoeff3 = (3/pi)*((DiameterRatio-1)/log(DiameterRatio));

MaxSpringForce = ((4*YoungModulus*10^3)/(1-PoissonRatio^2))*((MaterialThickness^3*SpringDeflection)/(ShapeCoeff1*OuterDiameter^2))...
                    *((InsideHeight/MaterialThickness-SpringDeflection/MaterialThickness)*(InsideHeight/MaterialThickness-SpringDeflection/...
                    2*MaterialThickness)+1);        % Unit = N

%% Clutch Calculations
N = 5;
CoFPlates = 0.16;
EffectiveRadius = 0.032995322;      % Unit = m
ClutchTorque = N*CoFPlates*LateralForcePressureRing*EffectiveRadius;                
%% Car Dimensional Calculations
WheelRadius = 9*25.4;       % Unit = mm

RLNormalLoad = CarWeight/4; % Unit = kg         ***FUNCTION OF LOAD TRANSFERS, NOT A CONSTANT***
RRNormalLoad = CarWeight/4; % Unit = kg         ***FUNCTION OF LOAD TRANSFERS, NOT A CONSTANT***

RLTractionLimit = RLNormalLoad*Gravity*CoFRoad;
RRTractionLimit = RRNormalLoad*Gravity*CoFRoad;

RLGroundTorque = RLTractionLimit*WheelRadius/1000;  % Unit = Nm
RRGroundTorque = RRTractionLimit*WheelRadius/1000;  % Unit = Nm
%% Output
format short g
clc
% TorqueLeft = 0.5*InputTorque + 0.5*TorqueDiff;
% TorqueRight = 0.5*InputTorque - 0.5*TorqueDiff;
% TorqueDiffFast
% TorqueDiffSlow
% YawMoment = (RearTrackWidth / TireRadius) * TorqueDiff;