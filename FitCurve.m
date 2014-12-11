function [fitresult, gof] = FitCurve(RPM, Torque)
%CREATEFIT(RPM,TORQUE)
%  Create a fit.
%
%  Data for 'EngineTorqueCurve' fit:
%      X Input : RPM
%      Y Output: Torque
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 04-Mar-2014 12:32:25


%% Fit: 'EngineTorqueCurve'.
[xData, yData] = prepareCurveData( RPM, Torque );

% Set up fittype and options.
ft = 'linearinterp';
opts = fitoptions( ft );
opts.Normalize = 'on';

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

          % Plot fit with data.
%figure( 'Name', 'Force Curve' );
%h = plot( fitresult, xData, yData );
%legend( h, 'Force vs. Velocity', 'Force Curve', 'Location', 'NorthEast' );
           % Label axes
%xlabel( 'Velocity' );
%ylabel( 'Force @ both wheels' );
%grid on


