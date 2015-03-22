%% Inputs
PowerAngle = 45;        % Unit = Degrees
CoastingAngle = 60;     % Unit = Degrees
PreloadTorque = 15;     % Unit = Nm
InputTorque = 38;       % Unit = Nm   ***FUNCTION OF RPM, NOT A CONSTANT***
FrontSprocketTeeth = 11;
RearSprocketTeeth = 40;
% DrivelineEfficiency = 0.94

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


%% Calculations
FDR = RearSprocketTeeth/FrontSprocketTeeth;
FinalDriveTorque = InputTorque*PrimaryRatio*Gear1Ratio*FDR;                 % Unit = Nm
ForcePressureRing = FinalDriveTorque/PressureRingRadius;                    % Unit = N
LateralForcePressureRing = tand(90-PowerAngle)*ForcePressureRing/2;         % Unit = N   %% DONE
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
CoFPlates = 0.16; % From http://www.engineeringtoolbox.com/friction-coefficients-d_778.html
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