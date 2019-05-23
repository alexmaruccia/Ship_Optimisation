function [Total_power, Power_prop, Power_aux_int, t_prop, T_prop, t_end_prop] = Total_power_evaluation_func_adv()

% Power required for propulsion
DataSheet = xlsread('Serenade AIS and ME 2018-12-16','Sensible Data');
Power_prop = DataSheet(:,3); %kw
t_prop = DataSheet(:,2); %s
T_prop = length(t_prop);
t_end_prop = t_prop(end);

% figure
% plot(t_prop,Power_prop/1000,'r')
% xlabel('Time [s]') 
% ylabel('Propulsion power demand [MW]') 
% title('Propulsion power demand')
% xlim([0 t_end_prop])
% grid on

% Power required for auxiliary
DataSheet = xlsread('Serenade auxiliary power');
powerDemand_aux = DataSheet(:,2);
t_aux = DataSheet(:,3);
T_aux = length(t_aux);
t_end_aux = t_aux(end);

% figure
% plot(t_aux,powerDemand_aux,'r')
% xlabel('Time [s]')
% ylabel('Auxiliary power demand [kW]')
% title('Auxiliary power demand')
% xlim([0 t_end_aux])
% grid on

%% 
t_1_peak_aux = 130080; %time peak starts
t_2_peak_aux = 132360; %time peak finishes
t_peak_medium_aux = mean([t_1_peak_aux t_2_peak_aux]); %point of referement for the interval of auxiliary power

t_prop_still = 18000; %in propulsion power: arriving to mariehamn - this point and t_peak_medium_aux are in the same time 
t_prop_zero = 0;
t_end_prop = t_prop(end); %already done

%% Building of the new intervals
DT_1_prop = t_prop_still - t_prop_zero; % first interval before the referment point (Delta in propulsion)
DT_2_prop = t_end_prop - t_prop_still; %second interval after the referement point (Delta in propulsion)

t_zero_aux_new = t_peak_medium_aux - DT_1_prop; %new time 0 for the auxiliary
t_end_aux_new = t_peak_medium_aux + DT_2_prop; %new time end for the auxiliary

t_aux_new = t_aux(t_aux >= t_zero_aux_new); % the time for auxiliary part are the same if the aux time >= t_zero_aux_new
t_aux_new = t_aux_new(t_aux_new < t_end_aux_new); % the propulsion end time is more than the auxiliary end time : I consider that the minor end time as the total time end
Dt_end_time = t_end_aux_new(end) - t_aux(end); %evaluation of the end time difference

%% evaluation of the first interval for the auxiliary to be mixed with the last interval of the propulopesion
t_aux_add = t_aux(t_aux < Dt_end_time) + t_aux(end); %the first new time cannot be zero; it is the last time + the new value
powerDemand_aux_add = powerDemand_aux(1:length(t_aux_add)); % the same thing done with time, is done for power demand
t_aux_new_complete = [t_aux_new; t_aux_add] - t_zero_aux_new;

Power_aux_new = powerDemand_aux(find(t_aux >= t_zero_aux_new));
Power_aux_add = powerDemand_aux(find(t_aux <= Dt_end_time));
Power_aux_new_complete = [Power_aux_new; Power_aux_add]; % the same profile of the auxiliary power demand, but starting and finishing in different times

% figure
% plot(t_aux_new_complete/3600,Power_aux_new_complete)
% xlabel('Time [h]')
% ylabel('Auxiliary power demand [kW]')
% title('Auxiliary power demand')
% xlim([0 t_aux_new_complete(end)/3600])
% grid on

%% I want to interpolate the auxiliary power, so I can add the aux power to the propulsion one

[t_aux_new_complete, index] = unique(t_aux_new_complete); 
Power_aux_int = interp1(t_aux_new_complete, Power_aux_new_complete(index), t_prop, 'spline');

%%
Total_power = Power_aux_int + Power_prop;

% figure
% plot(t_prop/3600,Power_prop/1000,'k')
% hold on
% plot(t_prop/3600,Power_aux_int/1000,'b')
% hold on
% plot(t_prop/3600,Total_power/1000,'r')
% xlabel('Time [h]')
% ylabel('Power demand [MW]')
% title('Propulsion and Auxiliary Power demand')
% legend ('Propulsion power','Auxiliary power','Total power')
% xlim([0 t_prop(end)/3600])
% grid on

end

