function [fitresult, gof] = SurfaceFitting(x,y,z,fittype)

%SurfaceFitting 二维数据拟合
%   z是一个表
%   每个x对应z的每一列
%   每个y对应z的每一行
%   fittype：拟合类型
[xData, yData, zData] = prepareSurfaceData(x,y,z);
% Set up fittype and options.


ft = fittype;

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );
%[fitresult, gof] = fit( [x', y'], z, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', ['fitted ' inputname(3)] );
h = plot( fitresult, [xData, yData], zData );

legend( h, 'fitted data', [inputname(3) ' vs. ' inputname(1) ' ,' inputname(2)], 'Location', 'NorthEast', 'Interpreter', 'none' );

% Label axes
xlabel( inputname(1), 'Interpreter', 'none' );
ylabel( inputname(2), 'Interpreter', 'none' );
zlabel( ['fitted ' inputname(3)], 'Interpreter', 'none' );
grid on
view( -52.1, 33.3 );

end
