%% Major Car Parameter (INPUT)
CarMass = 203; %KG
DriverMass = 67; %KG
AeroMass = 10; %KG
TotalMass = CarMass + DriverMass +AeroMass;
WeightBiasFront=0.45;   %*****In progress***** Need to understand Tire load sensitivity and  how C.G location affections it
WeightBiasRear=1-WeightBiasFront; %*****In progress***** Need to understand Tire load sensitivity and  how C.G location affections it

%Program Variables
format long g
NumVariable=12;
segmentLength = 0.1;

%AeroDynamics 
LiftCoefficient = 0.1;
DragCoefficient = 0.6657; 
AeroBalance = 0.5;
CarMaxVel=0;% Might be Able to remove this. This is for declaration only. Do not change this value.
AeroVel= 0: 10 : 140;
AeroMaxVel = zeros(4,1);

%Initial Conditions *****Need more study on this***** (Input)
LaunchVel= 50;
LaunchGear=2;

%Brakes Efficency(Input) 
%Because this is a point particle simulation, This is to take into account
%the reduction in braking due to load transfer
BrakeEff=0.5;

%Driver Behaviour
DriverThrottle= 1; %Percentage of Full Throttle
DriverBrake=0.5; %Not used yet

%RacingTrack Details (Input)
%This part is included so as to have changing radius in a corner exit and
%corner entry.Noted that the Avaliable movement must be a positive value to
%give acceptable results
RaceTrackWidth=4;
CarTrackWidth=1.190;
CarToConeClearance=0.6;
AvaliableMovement=RaceTrackWidth-CarTrackWidth-2*CarToConeClearance;

SegmentCounter=0;
PercentageComplete=0;

%Gear ratios (INPUT)
FDR = 3.27; 


TotalGearRatio1= FDR * 5.806;
TotalGearRatio2= FDR * 4.222;
TotalGearRatio3= FDR * 3.519;
TotalGearRatio4= FDR * 3.049;
TotalGearRatio5= FDR * 2.754;
TotalGearRatio6= FDR * 2.551;
DriveLineEff = 0.9; 
ShiftTime = 0.3;

%TireParameter 
TireRadius = 0.2286;
muLong2 = 2;
muLat2 = 2;
muLong = muLong2 - 0.084*TotalMass/2*9.81*10^-3; %Tire load sensitivity **** Need more study on this**** Need to change this MuLong because tire usually weaker in the long direction. 
muLat = muLat2 - 0.084*TotalMass/2*9.81*10^-3;  %Tire load sensitivity **** Need more study on this****Cureent Value is assume a Lat g of 1.38

%% Import the data

%[~, ~, raw] = xlsread('C:\Users\FSAE Shared\Documents\MATLAB\R14 Simulation\14 NUS lapsim\R14EngineTorqueCurve.xlsx','Sheet1','A1:B87');
 [~, ~, raw] = xlsread('C:\Users\Hong Han\Desktop\Dropbox\FSAE Hong Han\14 NUS lapsim\R14EngineTorqueCurve.xlsx','Sheet1','A1:B87');
%% Create output variable
data = reshape([raw{:}],size(raw));

%% Allocate imported array to column variable names
EngineRPM = data(:,1);
EngineTorque = data(:,2);

%% Clear temporary variables
clearvars data raw;


%% Drive Velocity at different gears 
Gear1Velocity = EngineRPM*2*pi/60/TotalGearRatio1*TireRadius*3.6;
Gear2Velocity = EngineRPM*2*pi/60/TotalGearRatio2*TireRadius*3.6;
Gear3Velocity = EngineRPM*2*pi/60/TotalGearRatio3*TireRadius*3.6;
Gear4Velocity = EngineRPM*2*pi/60/TotalGearRatio4*TireRadius*3.6;
Gear5Velocity = EngineRPM*2*pi/60/TotalGearRatio5*TireRadius*3.6;
Gear6Velocity = EngineRPM*2*pi/60/TotalGearRatio6*TireRadius*3.6;

%% Gear Force at wheels **Both Wheels**
Gear1Force = DriverThrottle*DriveLineEff*EngineTorque * TotalGearRatio1 / TireRadius;
Gear2Force = DriverThrottle*DriveLineEff*EngineTorque * TotalGearRatio2 / TireRadius;
Gear3Force = DriverThrottle*DriveLineEff*EngineTorque * TotalGearRatio3 / TireRadius;
Gear4Force = DriverThrottle*DriveLineEff*EngineTorque * TotalGearRatio4 / TireRadius;
Gear5Force = DriverThrottle*DriveLineEff*EngineTorque * TotalGearRatio5 / TireRadius;
Gear6Force = DriverThrottle*DriveLineEff*EngineTorque * TotalGearRatio6 / TireRadius;

%% Fitting of different force velocity curves -Doing this will allow as to get force at wheelS(both Wheels) by inputing the current speed. Example "Force@wheel = Gear2ForceCurve(50)
Gear1ForceCurve = FitCurve(Gear1Velocity,Gear1Force);
Gear2ForceCurve = FitCurve(Gear2Velocity,Gear2Force);
Gear3ForceCurve = FitCurve(Gear3Velocity,Gear3Force);
Gear4ForceCurve = FitCurve(Gear4Velocity,Gear4Force);
Gear5ForceCurve = FitCurve(Gear5Velocity,Gear5Force);
Gear6ForceCurve = FitCurve(Gear6Velocity,Gear6Force);

%% Find Ideal Shift Drive Velocity
% Shifting point from 1 to 2
if isempty(max(intersections(Gear1Velocity,Gear1Force,Gear2Velocity,Gear2Force,1)))==1
GearOnetoTwo = max(Gear1Velocity);
GearTwotoOne = max(Gear1Velocity);
else
GearOnetoTwo = max(intersections(Gear1Velocity,Gear1Force,Gear2Velocity,Gear2Force,1));
GearTwotoOne = max(intersections(Gear1Velocity,Gear1Force,Gear2Velocity,Gear2Force,1));
end
% Shifting point from 2 to 3
if isempty(max(intersections(Gear2Velocity,Gear2Force,Gear3Velocity,Gear3Force,1)))==1
GearTwotoThree = max(Gear2Velocity);
GearThreetoTwo = max(Gear2Velocity);
else
GearTwotoThree = max(intersections(Gear2Velocity,Gear2Force,Gear3Velocity,Gear3Force,1));
GearThreetoTwo = max(intersections(Gear2Velocity,Gear2Force,Gear3Velocity,Gear3Force,1));
end

% Shifting point from 3 to 4
if isempty(max(intersections(Gear3Velocity,Gear3Force,Gear4Velocity,Gear4Force,1)))==1
GearThreetoFour = max(Gear3Velocity);
GearFourtoThree = max(Gear3Velocity);
else
GearThreetoFour = max(intersections(Gear3Velocity,Gear3Force,Gear4Velocity,Gear4Force,1));
GearFourtoThree = max(intersections(Gear3Velocity,Gear3Force,Gear4Velocity,Gear4Force,1));
end

% Shifting point from 4 to 5
if isempty(max(intersections(Gear4Velocity,Gear4Force,Gear5Velocity,Gear5Force,1)))==1
GearFourtoFive = max(Gear4Velocity);
GearFivetoFour = max(Gear4Velocity);
else
GearFourtoFive = max(intersections(Gear4Velocity,Gear4Force,Gear5Velocity,Gear5Force,1));
GearFivetoFour = max(intersections(Gear4Velocity,Gear4Force,Gear5Velocity,Gear5Force,1));
end

% Shifting point from 5 to 6
if isempty(max(intersections(Gear5Velocity,Gear5Force,Gear6Velocity,Gear6Force,1)))==1
GearFivetoSix = max(Gear5Velocity);
GearSixtoFive = max(Gear5Velocity);
else
GearFivetoSix = max(intersections(Gear5Velocity,Gear5Force,Gear6Velocity,Gear6Force,1));
GearSixtoFive = max(intersections(Gear5Velocity,Gear5Force,Gear6Velocity,Gear6Force,1));
end

%%
%Shifting (INPUT)
UpShiftVelMat=[GearOnetoTwo, GearTwotoThree, GearThreetoFour, GearFourtoFive, GearFivetoSix, 9999];
DownShiftVelMat=[-9999, GearTwotoOne, GearThreetoTwo, GearFourtoThree, GearFivetoFour, GearSixtoFive];
CurrentGear =LaunchGear;
CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
CurrentDownShiftVel= DownShiftVelMat(CurrentGear);

%% Calculate Car Max Speed *****Not A robust logic to calculate vehicle max speed*****  Doesnt take into account high drag car with max speed of < 50km/hr but good enough for most of our cars
AeroDrag= DragCoefficient*(AeroVel/3.6).^2;
DragForceCurve= FitCurve(AeroVel,AeroDrag);

if(DragCoefficient==0)
CarMaxVel= max(Gear6Velocity);
else

if DragForceCurve(max(Gear6Velocity)) < Gear6ForceCurve(max(Gear6Velocity))
AeroMaxVel(1) = max(Gear6Velocity);
else
AeroMaxVel(1) = max(intersections(Gear6Velocity,Gear6Force,AeroVel,AeroDrag,1));
end
if DragForceCurve(max(Gear5Velocity)) < Gear5ForceCurve(max(Gear5Velocity))
AeroMaxVel(2) = max(Gear5Velocity);
else
AeroMaxVel(2) = max(intersections(Gear5Velocity,Gear5Force,AeroVel,AeroDrag,1));
end
if DragForceCurve(max(Gear4Velocity)) < Gear4ForceCurve(max(Gear4Velocity))
AeroMaxVel(3) = max(Gear4Velocity);
else
AeroMaxVel(3) = max(intersections(Gear4Velocity,Gear4Force,AeroVel,AeroDrag,1));
end
if DragForceCurve(max(Gear3Velocity)) < Gear3ForceCurve(max(Gear3Velocity))
AeroMaxVel(4) = max(Gear3Velocity);
else
AeroMaxVel(4) = max(intersections(Gear3Velocity,Gear3Force,AeroVel,AeroDrag,1));
end

end
CarMaxVel = max(AeroMaxVel);

syms Velocity; 

%% Running Program
details = xlsread('TrackDetails.xlsx', 'Acceleration');
track = 'Acceleration';
run MainSimulation

details = xlsread('TrackDetails.xlsx', 'Skidpad');
track = 'Skidpad';
run MainSimulation

details = xlsread('TrackDetails.xlsx', 'Slalom');
track = 'Slalom';
run MainSimulation

details = xlsread('TrackDetails.xlsx', 'UTurn');
track = 'UTurn';
run MainSimulation