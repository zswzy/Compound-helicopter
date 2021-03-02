function [fitresult, gof] = createFitL_PR(theta_0PR, L_PR)
%CREATEFIT(THETA_0PR,L_PR)
%  Create a fit.
%
%  Data for 'L_PR_fitted' fit:
%      X Input : theta_0PR
%      Y Output: L_PR
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  另请参阅 FIT, CFIT, SFIT.

%  由 MATLAB 于 22-Oct-2020 21:49:58 自动生成


%% Fit: 'L_PR_fitted'.
[xData, yData] = prepareCurveData( theta_0PR, L_PR );

% Set up fittype and options.
ft = 'linearinterp';

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft );

% Plot fit with data.
figure( 'Name', 'L_PR_fitted' );
h = plot( fitresult, xData, yData );
legend( h, 'L_PR vs. theta_0PR', 'L_PR_fitted', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'theta_0PR', 'Interpreter', 'none' );
ylabel( 'L_PR', 'Interpreter', 'none' );
grid on


