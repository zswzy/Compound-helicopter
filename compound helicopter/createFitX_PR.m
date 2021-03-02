function [fitresult, gof] = createFitX_PR(theta_0PR, X_PR)
%CREATEFIT(THETA_0PR,X_PR)
%  Create a fit.
%
%  Data for 'X_PR_fitted' fit:
%      X Input : theta_0PR
%      Y Output: X_PR
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 22-Oct-2020 21:42:17 自动生成


%% Fit: 'X_PR_fitted'.
[xData, yData] = prepareCurveData( theta_0PR, X_PR );

% Set up fittype and options.
ft = 'linearinterp';

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'X_PR_fitted' );
h = plot( fitresult, xData, yData );
legend( h, 'X_PR vs. theta_0PR', 'X_PR_fitted', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'theta_0PR', 'Interpreter', 'none' );
ylabel( 'X_PR', 'Interpreter', 'none' );
grid on


