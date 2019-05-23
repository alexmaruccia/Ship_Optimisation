clc
clear all
close all

DataSheet = xlsread('Polar Diagram Lookup_tables');
True_wind=DataSheet(2:end,1); %values from 0 to 25 m/s
Angle_grade=DataSheet(1,2:end); %values from 0 to 360°
Thrust=DataSheet(2:end,2:end);

Angle_rad_mat=repmat(Angle_grade,26,1); % matrix for the interpolation (x axis)
True_wind_mat=repmat(True_wind,1,25); % matrix for the interpolation (y axis)

angle_val=0:1:360; % new knots where I want to interpolate (X), with difference of 1°
wind_val=0:0.1:True_wind(end);% new knots where I want to interpolate (Y), with difference of 0.1


Thrust_int = interp2(Angle_rad_mat,True_wind_mat,Thrust,angle_val,wind_val'); %interpolation of values

%plot with mesh
mesh(angle_val,wind_val,Thrust_int)
xlim([0,angle_val(end)])
xlabel('Angle [°]')
ylabel('True Wind Speed [m/s]')
zlabel ('Thrust [kW]')
title('Polar Diagram interpolated values in 3D')
grid on
grid minor


figure
contour(angle_val,wind_val,Thrust_int)
xlabel('Angle [°]')
ylabel('True Wind Speed [m/s]')
legend ('Thrust [kW]')
title('Polar Diagram interpolated values')
grid on
grid minor


