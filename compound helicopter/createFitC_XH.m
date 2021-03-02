function [fitresult, gof] = createFitC_XH(alpha_HS, alpha, C_XH)
%CREATEFIT1(ALPHA_HS,ALPHA,C_XH)
%  Create a fit.
%
%  Data for 'C_XH_fitted' fit:
%      X Input : alpha_HS
%      Y Input : alpha
%      Z Output: C_XH
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 22-Oct-2020 18:44:54 自动生成


%% Fit: 'C_XH_fitted'.
[xData, yData, zData] = prepareSurfaceData( alpha_HS, alpha, C_XH );

% Set up fittype and options.
ft = 'thinplateinterp';

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'C_XH_fitted' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'C_XH_fitted', 'C_XH vs. alpha_HS, alpha', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'alpha_HS', 'Interpreter', 'none' );
ylabel( 'alpha', 'Interpreter', 'none' );
zlabel( 'C_XH', 'Interpreter', 'none' );
grid on
view( -77.7, 50.3 );


