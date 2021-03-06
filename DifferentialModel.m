%% Import Data
EngineRPM = xlsread('R15EngineTorqueCurve.xlsx','Sheet1','A1:A74');
EngineTorque = xlsread('R15EngineTorqueCurve.xlsx','Sheet1','B1:B74');

%% Program Variables
format long g
segmentLength = 0.1;
ExportFileName = 'TransmissionModelResults.xlsx';

%% Major Car Parameters
Gravity = 9.81;
CarMass = 197; %KG
DriverMass = 67; %KG
AeroMass = 0; %KG
TotalMass = CarMass + DriverMass +AeroMass;
WeightBiasFront = 0.47;   %*****In progress***** Need to understand Tire load sensitivity and  how C.G location affections it
WeightBiasRear = 1 - WeightBiasFront; %*****In progress***** Need to understand Tire load sensitivity and  how C.G location affections it

CoFRoad = 1.1;

CGHeight = 315;              % Unit = mm
Wheelbase = 1555;            % Unit = mm
FrontTrackWidth = 1200;      % Unit = mm
RearTrackWidth = 1190;      % Unit = mm
TireRadius = 0.254;

% AeroDynamics
LiftCoefficient = 0.1;
DragCoefficient = 0.6657;
AeroBalance = 0.5;
CarMaxVel = 0;% Might be Able to remove this. This is for declaration only. Do not change this value.
AeroVel = 0: 10 : 140;
AeroMaxVel = zeros(4,1);
syms velocity, syms velocity1;

% Brakes Efficency
% Because this is a point particle simulation, This is to take into account the reduction in braking due to load transfer
BrakeEff = 0.5;

% Driver Behaviour
DriverThrottle = 1; %Percentage of Full Throttle
DriverBrake = 0.5; %Not used yet

% Racing Track Details
% This part is included so as to have changing radius in a corner exit and corner entry
% Noted that the Avaliable movement must be a positive value to give acceptable results
RaceTrackWidth = 4;
CarTrackWidth = 1.190;
CarToConeClearance = 0.6;
AvaliableMovement = RaceTrackWidth - CarTrackWidth - 2*CarToConeClearance;

% Gear Ratios
FrontSprocketTeeth = 11;
RearSprocketTeeth = 36;
FDR = RearSprocketTeeth/FrontSprocketTeeth;   % USING
TotalGearRatio1 = FDR * 5.806;
TotalGearRatio2 = FDR * 4.222;
TotalGearRatio3 = FDR * 3.519;
TotalGearRatio4 = FDR * 3.049;
TotalGearRatio5 = FDR * 2.754;
TotalGearRatio6 = FDR * 2.551;
DriveLineEff = 0.9;
ShiftTime = 0.2;

% Tire Parameters
muLong2 = 2;
muLat2 = 2;
muLong = muLong2 - 0.084*TotalMass/2*9.81*10^-3; %Tire load sensitivity **** Need more study on this****
% Need to change this MuLong because tire usually weaker in the long direction.
muLat = muLat2 - 0.084*TotalMass/2*9.81*10^-3;  %Tire load sensitivity **** Need more study on this****
% Current Value is assume a Lat g of 1.38

%% Differential Inputs
PowerAngle = 45;        % Unit = Degrees   % USING
CoastingAngle = 60;     % Unit = Degrees
PreloadTorque = 15;     % Unit = Nm        % USING
PressureRingRadius = 0.037594;  % Unit = m

N = 10; % USING
CoFPlates = 0.16; % From http://www.engineeringtoolbox.com/friction-coefficients-d_778.html
EffectiveRadius = 0.032995322;      % Unit = m

%% Belleville Spring Initialization
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
                
%% Drive Velocity at different gears
Gear1Velocity = EngineRPM * 2 * pi / 60 / TotalGearRatio1 * TireRadius * 3.6;
Gear2Velocity = EngineRPM * 2 * pi / 60 / TotalGearRatio2 * TireRadius * 3.6;
Gear3Velocity = EngineRPM * 2 * pi / 60 / TotalGearRatio3 * TireRadius * 3.6;
Gear4Velocity = EngineRPM * 2 * pi / 60 / TotalGearRatio4 * TireRadius * 3.6;
Gear5Velocity = EngineRPM * 2 * pi / 60 / TotalGearRatio5 * TireRadius * 3.6;
Gear6Velocity = EngineRPM * 2 * pi / 60 / TotalGearRatio6 * TireRadius * 3.6;

%% Gear Force at wheels **Both Wheels**
Gear1Force = DriverThrottle * EngineTorque * TotalGearRatio1 / TireRadius;
Gear2Force = DriverThrottle * EngineTorque * TotalGearRatio2 / TireRadius;
Gear3Force = DriverThrottle * EngineTorque * TotalGearRatio3 / TireRadius;
Gear4Force = DriverThrottle * EngineTorque * TotalGearRatio4 / TireRadius;
Gear5Force = DriverThrottle * EngineTorque * TotalGearRatio5 / TireRadius;
Gear6Force = DriverThrottle * EngineTorque * TotalGearRatio6 / TireRadius;

%% Fitting of different force velocity curves
% Get force at wheelS(both Wheels) by inputing the current speed. Example "Force@wheel = Gear2ForceCurve(50)
Gear1ForceCurve = FitCurve(Gear1Velocity,Gear1Force);
Gear2ForceCurve = FitCurve(Gear2Velocity,Gear2Force);
Gear3ForceCurve = FitCurve(Gear3Velocity,Gear3Force);
Gear4ForceCurve = FitCurve(Gear4Velocity,Gear4Force);
Gear5ForceCurve = FitCurve(Gear5Velocity,Gear5Force);
Gear6ForceCurve = FitCurve(Gear6Velocity,Gear6Force);

%% Find Ideal Shift Drive Velocity
% Shifting Point between Gear 1 and Gear 2
if isempty(max(intersections(Gear1Velocity,Gear1Force,Gear2Velocity,Gear2Force,1)))==1
    GearOnetoTwo = max(Gear1Velocity);
    GearTwotoOne = max(Gear1Velocity);
else
    GearOnetoTwo = max(intersections(Gear1Velocity,Gear1Force,Gear2Velocity,Gear2Force,1));
    GearTwotoOne = max(intersections(Gear1Velocity,Gear1Force,Gear2Velocity,Gear2Force,1));
end
% Shifting Point between Gear 2 and Gear 3
if isempty(max(intersections(Gear2Velocity,Gear2Force,Gear3Velocity,Gear3Force,1)))==1
    GearTwotoThree = max(Gear2Velocity);
    GearThreetoTwo = max(Gear2Velocity);
else
    GearTwotoThree = max(intersections(Gear2Velocity,Gear2Force,Gear3Velocity,Gear3Force,1));
    GearThreetoTwo = max(intersections(Gear2Velocity,Gear2Force,Gear3Velocity,Gear3Force,1));
end

% Shifting Point between Gear 3 and Gear 4
if isempty(max(intersections(Gear3Velocity,Gear3Force,Gear4Velocity,Gear4Force,1)))==1
    GearThreetoFour = max(Gear3Velocity);
    GearFourtoThree = max(Gear3Velocity);
else
    GearThreetoFour = max(intersections(Gear3Velocity,Gear3Force,Gear4Velocity,Gear4Force,1));
    GearFourtoThree = max(intersections(Gear3Velocity,Gear3Force,Gear4Velocity,Gear4Force,1));
end

% Shifting Point between Gear 4 and Gear 5
if isempty(max(intersections(Gear4Velocity,Gear4Force,Gear5Velocity,Gear5Force,1)))==1
    GearFourtoFive = max(Gear4Velocity);
    GearFivetoFour = max(Gear4Velocity);
else
    GearFourtoFive = max(intersections(Gear4Velocity,Gear4Force,Gear5Velocity,Gear5Force,1));
    GearFivetoFour = max(intersections(Gear4Velocity,Gear4Force,Gear5Velocity,Gear5Force,1));
end

% Shifting Point between Gear 5 and Gear 6
if isempty(max(intersections(Gear5Velocity,Gear5Force,Gear6Velocity,Gear6Force,1)))==1
    GearFivetoSix = max(Gear5Velocity);
    GearSixtoFive = max(Gear5Velocity);
else
    GearFivetoSix = max(intersections(Gear5Velocity,Gear5Force,Gear6Velocity,Gear6Force,1));
    GearSixtoFive = max(intersections(Gear5Velocity,Gear5Force,Gear6Velocity,Gear6Force,1));
end

%% Running Program
run Accel
run Skidpad
run Slalom
run UTurn
run Oval