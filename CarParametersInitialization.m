%%Major Car Parameter (INPUT)
CarMass = 203; %KG
DriverMass = 67; %KG
AeroMass = 10; %KG
TotalMass = CarMass + DriverMass +AeroMass;
WeightBiasFront = 0.45;   %*****In progress***** Need to understand Tire load sensitivity and  how C.G location affections it
WeightBiasRear = 1 - WeightBiasFront; %*****In progress***** Need to understand Tire load sensitivity and  how C.G location affections it

%AeroDynamics 
LiftCoefficient = 0.1;
%DragCoefficient = 0.6657; 
%AeroBalance = 0.5;
CarMaxVel = 0;

%Initial Conditions *****Need more study on this***** (Input)
LaunchVel = 50;
LaunchGear = 2;

muLong = 2 - 0.084*TotalMass/2*9.81*10^-3; %Tire load sensitivity **** Need more study on this**** Need to change this MuLong because tire usually weaker in the long direction. 
muLat = 2 - 0.084*TotalMass/2*9.81*10^-3;  %Tire load sensitivity **** Need more study on this****Cureent Value is assume a Lat g of 1.38