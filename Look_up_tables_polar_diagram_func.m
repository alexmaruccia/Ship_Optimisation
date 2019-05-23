function [wind_val,angle_val, Thrust_int] = Look_up_tables_polar_diagram_func()

DataSheet = xlsread('Polar Diagram Lookup_tables');
True_wind=DataSheet(2:end,1); %values from 0 to 25 m/s
Angle_grade=DataSheet(1,2:end); %values from 0 to 360°
Thrust=DataSheet(2:end,2:end);

Angle_rad_mat=repmat(Angle_grade,26,1); % matrix for the interpolation (x axis)
True_wind_mat=repmat(True_wind,1,25); % matrix for the interpolation (y axis)

angle_val=0:1:360; % new knots where I want to interpolate (X), with difference of 1°
wind_val=0:0.1:True_wind(end);% new knots where I want to interpolate (Y), with difference of 0.1


Thrust_int = interp2(Angle_rad_mat,True_wind_mat,Thrust,angle_val,wind_val'); %interpolation of values


end

