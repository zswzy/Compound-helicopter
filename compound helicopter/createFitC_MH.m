function [fitresult, gof] = createFitC_MH(alpha_HS, alpha, C_MH)
%CREATEFIT(ALPHA_HS,ALPHA,C_MH)
%  Create a fit.
%
%  Data for 'C_MH_fitted' fit:
%      X Input : alpha_HS
%      Y Input : alpha
%      Z Output: C_MH
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 22-Oct-2020 20:45:15 自动生成


%% Fit: 'C_MH_fitted'.
[xData, yData, zData] = prepareSurfaceData( alpha_HS, alpha, C_MH );

% Set up fittype and options.
ft = 'thinplateinterp';

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'C_MH_fitted' );
h = plot( fitresult, [xData, yData], zData );
legend( h, 'C_MH_fitted', 'C_MH vs. alpha_HS, alpha', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'alpha_HS', 'Interpreter', 'none' );
ylabel( 'alpha', 'Interpreter', 'none' );
zlabel( 'C_MH', 'Interpreter', 'none' );
grid on
view( -52.1, 33.3 );


