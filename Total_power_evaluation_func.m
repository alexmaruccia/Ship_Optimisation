function [Total_power, t_aux_new_complete, T, t_end] = Total_power_evaluation_func()
DataSheet = xlsread('Serenade AIS and ME 2018-12-16','Sensible Data');
Power_prop=DataSheet(:,3); %kw
t_prop=DataSheet(:,2); %s
T=length(t_prop);
t_end=t_prop(end);

% figure
% plot(t_prop,Power_prop/1000,'r')
% xlabel('Time [s]') 
% ylabel('Propulsion power demand [MW]') 
% title('Propulsion power demand')
% xlim([0 t_end])
% grid on

%% Power required for auxiliary
DataSheet = xlsread('Serenade auxiliary power');
powerDemand_aux=DataSheet(:,2);
t_aux=DataSheet(:,3);
T=length(t_aux);
t_end=t_aux(end);

% figure
% plot(t_aux,powerDemand_aux,'r')
% xlabel('Time [s]')
% ylabel('Auxiliary power demand [kW]')
% title('Auxiliary power demand')
% xlim([0 t_end])
% grid on

t_1_peak_aux = 130080;
t_2_peak_aux = 132360;
t_peak_medium_aux = mean([t_1_peak_aux t_2_peak_aux]); %point of referement for the auxiliary

t_prop_still = 18000;
t_prop_zero = 0;
t_prop_end = t_prop(end);

DT_1_prop = t_prop_still-t_prop_zero; % first interval before the referment point
DT_2_prop = t_prop_end-t_prop_still; %second interval after the referement point

t_zero_aux_new=t_peak_medium_aux-DT_1_prop; %new time 0 for the auxiliary
t_end_aux_new=t_peak_medium_aux+DT_2_prop; %new time end for the auxiliary
t_aux_new=t_aux(t_aux>=t_zero_aux_new);
t_aux_new=t_aux_new(t_aux_new<t_end_aux_new); % the propulsion end time is more than the auxiliary end time : I consider that the minor end time as the total time end
Dt_end_time=t_end_aux_new(end)-t_aux(end); %evaluation of the end time difference

%% evaluation of the first interval for the auxiliary to be mixed with the last interval of the propulasion
t_aux_add=t_aux(t_aux<Dt_end_time)+t_aux(end); %the first new time cannot be zero;
powerDemand_aux_add=powerDemand_aux(1:length(t_aux_add));
t_aux_new_complete=[t_aux_new; t_aux_add] - t_zero_aux_new;

power_aux_new=powerDemand_aux(find(t_aux >= t_zero_aux_new));
power_aux_add=powerDemand_aux(find(t_aux <= Dt_end_time));
Power_aux_new_complete= [power_aux_new; power_aux_add];

% figure
% plot(t_aux_new_complete/3600,Power_aux_new_complete)
% xlabel('Time [h]')
% ylabel('Auxiliary power demand [kW]')
% title('Auxiliary power demand')
% xlim([0 t_aux_new_complete(end)/3600])
% grid on
%%
Power_prop_int = interp1(t_prop,Power_prop,t_aux_new_complete,'spline');
Total_power = Power_aux_new_complete+Power_prop_int; %kW
T = length(t_aux_new_complete);
t_end=t_aux_new_complete(end)/3600; %h
% figure
% plot(t_aux_new_complete,Power_prop_int/1000,'b')
% hold on
% plot(t_aux_new_complete,Power_aux_new_complete/1000,'r')
% hold on
% plot(t_aux_new_complete,Total_power/1000,'k')
% xlabel('Time [h]')
% ylabel('Power demand [MW]')
% title('Propulsion and Auxiliary Power demand')
% legend ('Propulsion power','Auxiliary power','Total power')
% xlim([0 t_aux_new_complete(end)])
% grid on
end

