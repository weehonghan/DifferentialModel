%% Data Initialization
rows = size(details, 1);
corners = rows - sum(details(:,1));

numSections = sum(details(:,2));
numSegments = (numSections / segmentLength) + 1 ; 

SegmentNumber = zeros(numSegments, 1);
Radius = zeros(numSegments, 1);
MaxVelocity = zeros(numSegments, 1);
CurrentVelocity = zeros(numSegments, 1);
Gear = zeros(numSegments, 1);
Acceleration = zeros(numSegments, 1);
EngineTorque = zeros(numSegments, 1);
TractiveForce = zeros(numSegments, 1);
ShiftTimeRemain = zeros(numSegments, 1);
SegmentTime = zeros(numSegments, 1);
AccumulatedTime = zeros(numSegments, 1);
Distance = zeros(numSegments, 1);

accumulatedSegment = segmentLength;
segmentIndex = 1;

% Initialize Launch Condition
SegmentNumber(1) = 1;
Radius(1) = 999999;
MaxVelocity(1) = CarMaxVel;
CurrentVelocity(1) = LaunchVel;
Gear(1) = LaunchGear;
Acceleration(1) = 9.81; %Assume 1g accerlation **doesnt really matters
CurrentVelocity(2) = LaunchVel;

% Initialize Segment Number and Radius
for i = 2:numSegments
    SegmentNumber(i) = i;
    if (accumulatedSegment >= (details(segmentIndex, 2)+0.00001))
        segmentIndex = segmentIndex + 1;
        accumulatedSegment = segmentLength;
    end
    Radius(i) = details(segmentIndex, 3); %#ok<*SAGROW>
    accumulatedSegment = accumulatedSegment + segmentLength;
end

% L = matrix
% D = Start
% E = End

% Initialize CornerEntry Matrix
CornerEntryMat = zeros(rows, 2);
start = 2;
End = 1;
kk = 1;
for k = 1:rows
    if ( details(k, 3) ~= 999999 )
        CornerEntryMat(kk, 1) = start;
    end
    start = start + ( details(k, 2) / segmentLength );
    End = End + ( details(k, 2) / segmentLength );
    if ( details(k, 3) ~= 999999 )
        CornerEntryMat(kk, 2) = End;
        kk = kk + 1;
    end;
end

for iii=1:corners
   MidCornerIndex=round((CornerEntryMat(iii,1)+ CornerEntryMat(iii,2))/2);
   CornerEntryGradient=AvaliableMovement/(MidCornerIndex-CornerEntryMat(iii,1));
   CornerExitGradient=0.15; %Need to change this crude assumption. This is assuming that the radius of turn changes linearly at corner exit.
   % The actual relationship is exponential in nature and it differs from corner to corner 
   %CornerExitGradient=AvaliableMovement/(CornerEntryMat(iii,2)-MidCornerIndex);
    
   for InnerLoop = CornerEntryMat(iii,1):  MidCornerIndex-1 
   Radius(InnerLoop+1)= Radius(InnerLoop) - CornerEntryGradient;
   end
   
   for InnerLoop2 = MidCornerIndex:CornerEntryMat(iii,2)-1
   Radius(InnerLoop2+1)= Radius(InnerLoop2) + CornerExitGradient;    
   end   
end

for j = 2:numSegments
    if (Radius(j) == 999999)
        MaxVelocity(j) = CarMaxVel;           
    elseif (Radius(j) == Radius(j-1))
         MaxVelocity(j)= MaxVelocity(j-1);
    else
        MaxVelocity(j) = 3.6 * vpasolve(-TotalMass*Velocity^2/Radius(j) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,[0 150]);
       %Lap(j, 3) = 3.6 * vpasolve(-TotalMass*Velocity^2/Lap(j, 2) + muLat*(TotalMass*9.814 + LiftCoefficient * Velocity^2),Velocity,Lap(j-1, 3));
    end
end

%% Maximum speed assignment
syms Velocity1; %****Move to the top****
counter=0;
PreviousVelStore=0;
for i=1:corners
   
CornerEntryIndex = CornerEntryMat(i,1);
    
while (MaxVelocity(CornerEntryIndex) < CarMaxVel) && (CornerEntryIndex>2)
        PreviousVelStore = 3.6*vpasolve(-Velocity1^2 + (MaxVelocity(CornerEntryIndex)/3.6)^2 + 2*segmentLength*(((BrakeEff  *muLong*(1-((Velocity1)^2/(Radius(CornerEntryIndex)*(9.814+LiftCoefficient*(Velocity1)^2)*muLat))^2)^0.5) * (TotalMass*9.814 + LiftCoefficient * (Velocity1)^2) + DragCoefficient * (Velocity1)^2)/TotalMass),(MaxVelocity(CornerEntryIndex)/3.6));
    if PreviousVelStore <  MaxVelocity(CornerEntryIndex-1)    
    MaxVelocity(CornerEntryIndex-1) = PreviousVelStore;
    
    else 
    break
    end
    %a= F/TotalMass;
    %F = FrictionLong * (TotalMass*9.814 + LiftCoefficient * (Velocity1)^2) + DragCoefficient * (Velocity1)^2;
    %FrictionLong = muLong*(1-((Velocity1)^2/(Lap(CornerEntryIndex,2)*(9.814+LiftCoefficient*(Velocity1)^2)*muLat))^2)^0.5;
    
    %F = (muLong*(1-((Velocity1)^2/(Lap(CornerEntryIndex,2)*(9.814+LiftCoefficient*(Velocity1)^2)*muLat))^2)^0.5) * (TotalMass*9.814 + LiftCoefficient * (Velocity1)^2) + DragCoefficient * (Velocity1)^2; 
    %a= ((muLong*(1-((Velocity1)^2/(Lap(CornerEntryIndex,2)*(9.814+LiftCoefficient*(Velocity1)^2)*muLat))^2)^0.5) * (TotalMass*9.814 + LiftCoefficient * (Velocity1)^2) + DragCoefficient * (Velocity1)^2)/TotalMass;
    %Lap(CornerEntryIndex-1,3)= vpasolve(-Velocity1^2 + (Lap(CornerEntryIndex,3))^2 + 2*SegmentLength*(((muLong*(1-((Velocity1)^2/(Lap(CornerEntryIndex,2)*(9.814+LiftCoefficient*(Velocity1)^2)*muLat))^2)^0.5) * (TotalMass*9.814 + LiftCoefficient * (Velocity1)^2) + DragCoefficient * (Velocity1)^2)/TotalMass);

counter=counter+1;
CornerEntryIndex= CornerEntryIndex-1;

end
end

%% Main Simulation
for iSegment = 2:numSegments-1
    if CurrentVelocity(iSegment) >= MaxVelocity(iSegment)
        %Decelerate Accordingly to maxVelocity
        CurrentVelocity(iSegment)=MaxVelocity(iSegment);
        %CalculateSegmentTime and Remaining Shift Time
        SegmentTime(iSegment) = 2*segmentLength/((CurrentVelocity(iSegment)/3.6)+(CurrentVelocity(iSegment-1)/3.6));
        
        %% Shifting
        if ShiftTimeRemain(iSegment-1) > 0 %If there is remaining Shift time
            ShiftTimeRemain(iSegment) = ShiftTimeRemain(iSegment-1) - SegmentTime(iSegment); %New Shift time remaining = Previous Shift time remaining - segment time
            else
            ShiftTimeRemain(iSegment) = 0;
        end
        
        
        if(CurrentVelocity(iSegment)<= CurrentDownShiftVel && CurrentVelocity(iSegment-1) > CurrentDownShiftVel) %DownShifitng criteria
            CurrentGear = CurrentGear-1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(iSegment)=0;
        elseif (CurrentVelocity(iSegment-1)< CurrentUpShiftVel && CurrentVelocity(iSegment) >= CurrentUpShiftVel) %UpShifitng criteria
            CurrentGear = CurrentGear+1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(iSegment)=ShiftTime;
        end
        
        %Log current gear
        Gear(iSegment)=CurrentGear;
        
        %Calculating next Velocity
        if MaxVelocity(iSegment+1) <= MaxVelocity(iSegment)
            CurrentVelocity(iSegment+1) = MaxVelocity(iSegment+1);
        else
            CurrentVelocity(iSegment+1) = MaxVelocity(iSegment);
        end
        %Calculating  current acceleration
        Acceleration(iSegment) = ((CurrentVelocity(iSegment+1)/3.6)^2 -  (CurrentVelocity(iSegment)/3.6)^2)/(2*segmentLength);
        TractiveForce(iSegment) = TotalMass*Acceleration(iSegment);
        SegmentCounter=SegmentCounter+1;
        PercentageComplete =SegmentCounter/numSegments*100;

    else %Acceleration
        %CalculateSegmentTime and Remaining Shift Time
        SegmentTime(iSegment)= 2*segmentLength/((CurrentVelocity(iSegment)/3.6)+(CurrentVelocity(iSegment-1)/3.6));
        %Acceleration
        %% Shifting
        if ShiftTimeRemain(iSegment-1) > 0 %If there is remaining Shift time
            ShiftTimeRemain(iSegment) = ShiftTimeRemain(iSegment-1) - SegmentTime(iSegment); %New Shift time remaining = Previous Shift time remaining - segment time
        else
            ShiftTimeRemain(iSegment) = 0;
        end
        
        if(CurrentVelocity(iSegment)<= CurrentDownShiftVel && CurrentVelocity(iSegment-1) > CurrentDownShiftVel) %DownShifitng criteria
            CurrentGear = CurrentGear-1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(iSegment)=0;
        elseif (CurrentVelocity(iSegment-1)< CurrentUpShiftVel && CurrentVelocity(iSegment) >= CurrentUpShiftVel) %UpShifitng criteria
            CurrentGear = CurrentGear+1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(iSegment)=ShiftTime;
        end
        
        %Log current gear
        Gear(iSegment)=CurrentGear;
        
        %Calculate and log engine force
        switch(CurrentGear)
            case 1
                EngineTorque(iSegment)=Gear1ForceCurve(CurrentVelocity(iSegment));
            case 2
                EngineTorque(iSegment)=Gear2ForceCurve(CurrentVelocity(iSegment));
            case 3
                EngineTorque(iSegment)=Gear3ForceCurve(CurrentVelocity(iSegment));
            case 4
                EngineTorque(iSegment)=Gear4ForceCurve(CurrentVelocity(iSegment));
            case 5
                EngineTorque(iSegment)=Gear5ForceCurve(CurrentVelocity(iSegment));
            case 6
                EngineTorque(iSegment)=Gear6ForceCurve(CurrentVelocity(iSegment));
        end
        
        %Calculate avaliable tractive force 
        TractiveForce(iSegment) = ((muLong)*(1-(((CurrentVelocity(iSegment)/3.6)^4)/(Radius(iSegment)*(9.81+LiftCoefficient*((CurrentVelocity(iSegment)/3.6)^2)/TotalMass))^2/muLat^2)^0.5) * (TotalMass*9.814*WeightBiasRear + LiftCoefficient*((CurrentVelocity(iSegment)/3.6)^2)*AeroBalance)); 
        
        %Calculate Acceleration
        if ShiftTimeRemain(iSegment)>0
            Acceleration(iSegment) = 0;
        elseif TractiveForce(iSegment)>= EngineTorque(iSegment)
            Acceleration(iSegment) = (EngineTorque(iSegment)-DragCoefficient*(CurrentVelocity(iSegment)/3.6)^2)/TotalMass;
        else
            Acceleration(iSegment) = (TractiveForce(iSegment)-DragCoefficient*(CurrentVelocity(iSegment)/3.6)^2)/TotalMass;
        end
        
        %Calculate next velocity
        CurrentVelocity(iSegment+1) = ((((CurrentVelocity(iSegment)/3.6)^2)+2*Acceleration(iSegment)*segmentLength)^0.5)*3.6;
        SegmentCounter=SegmentCounter+1;
        PercentageComplete=SegmentCounter/numSegments*100;
    end
    
    AccumulatedTime(iSegment)= AccumulatedTime(iSegment-1)+SegmentTime(iSegment);
    Distance(iSegment)= (SegmentNumber(iSegment)-1)*segmentLength;
    
end

%%Ending Conditions

if CurrentVelocity(numSegments) >= MaxVelocity(numSegments)
        %Decelerate Accordingly to maxVelocity
        CurrentVelocity(numSegments)=MaxVelocity(numSegments);
        %CalculateSegmentTime and Remaining Shift Time
        SegmentTime(numSegments) = 2*segmentLength/((CurrentVelocity(numSegments)/3.6)+(CurrentVelocity(numSegments-1)/3.6));
        
        %% Shifting
        if ShiftTimeRemain(numSegments-1) > 0 %If there is remaining Shift time
            ShiftTimeRemain(numSegments) = ShiftTimeRemain(numSegments-1) - SegmentTime(numSegments); %New Shift time remaining = Previous Shift time remaining - segment time
            else
            ShiftTimeRemain(numSegments) = 0;
        end
        
        
        if(CurrentVelocity(numSegments)<= CurrentDownShiftVel && CurrentVelocity(numSegments-1) > CurrentDownShiftVel) %DownShifitng criteria
            CurrentGear = CurrentGear-1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(numSegments)=0;
        elseif (CurrentVelocity(numSegments-1)< CurrentUpShiftVel && CurrentVelocity(numSegments) >= CurrentUpShiftVel) %UpShifitng criteria
            CurrentGear = CurrentGear+1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(numSegments)=ShiftTime;
        end
        
        %Log current gear
        Gear(numSegments)=CurrentGear;
        
        
        %Calculating  current acceleration
        Acceleration(numSegments) = Acceleration(numSegments-1);
        TractiveForce(numSegments) = TotalMass*Acceleration(numSegments);
        
    else %Acceleration
        %CalculateSegmentTime and Remaining Shift Time
        SegmentTime(numSegments)= 2*segmentLength/((CurrentVelocity(numSegments)/3.6)+(CurrentVelocity(numSegments-1)/3.6));
        %Acceleration
        %% Shifting
        if ShiftTimeRemain(numSegments-1) > 0 %If there is remaining Shift time
            ShiftTimeRemain(numSegments) = ShiftTimeRemain(numSegments-1) - SegmentTime(numSegments); %New Shift time remaining = Previous Shift time remaining - segment time
        else
            ShiftTimeRemain(numSegments) = 0;
        end
        
        if(CurrentVelocity(numSegments)<= CurrentDownShiftVel && CurrentVelocity(numSegments-1) > CurrentDownShiftVel) %DownShifitng criteria
            CurrentGear = CurrentGear-1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(numSegments)=0;
        elseif (CurrentVelocity(numSegments-1)< CurrentUpShiftVel && CurrentVelocity(numSegments) >= CurrentUpShiftVel) %UpShifitng criteria
            CurrentGear = CurrentGear+1;
            CurrentUpShiftVel= UpShiftVelMat(CurrentGear);
            CurrentDownShiftVel= DownShiftVelMat(CurrentGear);
            ShiftTimeRemain(numSegments)=ShiftTime;
        end
        
        %Log current gear
        Gear(numSegments)=CurrentGear;
        
        %Calculate and log engine force
        switch(CurrentGear)
            case 1
                EngineTorque(numSegments)=Gear1ForceCurve(CurrentVelocity(numSegments));
            case 2
                EngineTorque(numSegments)=Gear2ForceCurve(CurrentVelocity(numSegments));
            case 3
                EngineTorque(numSegments)=Gear3ForceCurve(CurrentVelocity(numSegments));
            case 4
                EngineTorque(numSegments)=Gear4ForceCurve(CurrentVelocity(numSegments));
            case 5
                EngineTorque(numSegments)=Gear5ForceCurve(CurrentVelocity(numSegments));
            case 6
                EngineTorque(numSegments)=Gear6ForceCurve(CurrentVelocity(numSegments));
        end
        
        %Calculate avaliable tractive force
        TractiveForce(numSegments) = ((muLong)*(1-(((CurrentVelocity(numSegments)/3.6)^4)/(Radius(numSegments)*(9.81+LiftCoefficient*((CurrentVelocity(numSegments)/3.6)^2)/TotalMass))^2/muLat^2)^0.5) * (TotalMass*9.814*WeightBiasRear + LiftCoefficient*((CurrentVelocity(numSegments)/3.6)^2)*AeroBalance)); 
        
        %Calculate Acceleration
        if ShiftTimeRemain(numSegments)>0
            Acceleration(numSegments) = 0;
        elseif TractiveForce(numSegments)>= EngineTorque(numSegments)
            Acceleration(numSegments) = (EngineTorque(numSegments)-DragCoefficient*(CurrentVelocity(numSegments)/3.6)^2)/TotalMass;
        else
            Acceleration(numSegments) = (TractiveForce(numSegments)-DragCoefficient*(CurrentVelocity(numSegments)/3.6)^2)/TotalMass;
        end       
end
AccumulatedTime(numSegments)= AccumulatedTime(numSegments-1)+SegmentTime(numSegments);
Distance(numSegments)= (SegmentNumber(numSegments)-1)*segmentLength;

%% Graph Plotting
figure
plot(AccumulatedTime, CurrentVelocity)
title('Figure 1')
xlabel('AccumulatedTime')
ylabel('CurrentVelocity')
figure
plot(Distance, CurrentVelocity)
title('Figure 2')
xlabel('Distance')
ylabel('CurrentVelocity')

% Save To Excel
ExportFileName = 'TransmissionModelResults.xlsx';
xlswrite(ExportFileName, SegmentNumber, track, 'A2');
xlswrite(ExportFileName, Radius, track, 'B2');
xlswrite(ExportFileName, MaxVelocity, track, 'C2');
xlswrite(ExportFileName, CurrentVelocity, track, 'D2');
xlswrite(ExportFileName, Gear, track, 'E2');
xlswrite(ExportFileName, Acceleration, track, 'F2');
xlswrite(ExportFileName, EngineTorque, track, 'G2');
xlswrite(ExportFileName, TractiveForce, track, 'H2');
xlswrite(ExportFileName, ShiftTimeRemain, track, 'I2');
xlswrite(ExportFileName, SegmentTime, track, 'J2');
xlswrite(ExportFileName, AccumulatedTime, track, 'K2');
xlswrite(ExportFileName, Distance, track, 'L2');