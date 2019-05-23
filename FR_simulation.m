clc
clear all
close all

%% Evaluation of the FR's effect
% data of time from data (only time steps are needed)
[Total_power, Power_prop_demand, Power_aux_demand, t, T, t_end] = Total_power_evaluation_func_adv();

% sin angular velocity
w =  2*pi/t_end ;

% maximum wind speed = 25 m/s; min = 5 m/s
max_wind = 25 ;
min_wind = 10 ;
Y0 = (max_wind + min_wind)/2; % medium value
A0 = 2*(max_wind - Y0); % Amplitude

wind = A0/2*cos(w*t - 140) + Y0; % cos for wind
% wind = 20*ones(T,1); % cost for wind

% sin angle cosine
angle = 180*cos(w*t - 60) + 180; %cos for angle

% plot wind and angle
figure
yyaxis left
plot(t/3600,wind)
xlabel('Time [h]')
ylabel('True wind speed [m/s]')
xlim([0 t_end/3600])
ylim([0 30])
grid on
grid minor
hold on
yyaxis right
plot (t/3600, angle)
ylabel ('Relative angle [°]')
title ('True wind speed and relative angle')

% evaluation of the interpoleted values and the resulting thrust: creation of the interpoleded points with the lookup tables
[wind_val, angle_val, Thrust_int] = Look_up_tables_polar_diagram_func();

wind = round (wind);
angle = round (angle);
idx_wind = zeros (length(wind),1);

for i= 1:length(wind)
    idx_wind(i) = find (wind_val == wind(i));
end

idx_angle = zeros (length(angle),1);

for i= 1:length(angle)
    idx_angle(i) = find (angle_val == angle(i));
end

idx = sub2ind( size(Thrust_int), idx_wind, idx_angle); % creation of coordinates for evaluating the thrust

Thrust = Thrust_int(idx); % evaluation of th thrust supplied (as power, in kW)

% Considering a power decreasing of 20% when the ship's speed is about 15 knots
Thrust([1:find(t == 8940)]) = Thrust([1:find(t == 8940)])*0.8;

% Considering a power decreasing of 50% when the ship's speed is zero
Thrust([find(t == 56220):end]) = Thrust([find(t == 56220):end])*0.5;

% plot of the thrust
figure
plot(t/3600,Thrust)
title ('Flettner rotor thrust')
xlabel('Time [h]')
ylabel('Thrust Power [kW]')
xlim([0 t_end/3600])
grid on
grid minor

% effect on the propulsion demand
[Total_power, Power_prop_demand, Power_aux_demand, t, T, t_end] = Total_power_evaluation_func_adv();

Power_prop_demand_new = Power_prop_demand - Thrust; % evaluation of the net value of power demand
Power_prop_demand_new(Power_prop_demand_new < 0) = 0; % not negative power demand

figure
plot(t/3600,Power_prop_demand/1000,'r')
hold on
plot(t/3600,Thrust/1000,'g')
plot(t/3600,Power_prop_demand_new/1000,'b')
xlabel('Time [h]')
ylabel('Propulsion power demand with FR [MW]')
title('Propulsion power demand with FR')
xlim([0 t_end/3600])
legend ('Original power demand', 'Contribution of the FR', 'Net power demand')
grid on
grid minor

%% Model

time_sim = 300;
MAX_ENGINE_POWER_1 = 8125; % in kW
MAX_ENGINE_POWER_2 = 7500;
MAX_ENGINE_POWER_3 = 7500;
MAX_ENGINE_POWER_4 = 8125;

MIN_ENGINE_POWER_14 = 0.2*MAX_ENGINE_POWER_1; 
MIN_ENGINE_POWER_23 = 0.2*MAX_ENGINE_POWER_2;

TIME_STEP_DURATION = t(2) - t(1); % 2 minutes
START_GRAMS = 3500;

figure
plot(t/3600,Power_prop_demand_new/1000,'r')
xlabel('Time [h]')
ylabel('Propulsion power demand [MW]')
title('Propulsion power demand')
xlim([0 t_end/3600])
grid on
grid minor

Consumption = xlsread('HFO_SFOC_ME');
Load=Consumption(:,1);
SFOC=Consumption(:,2);

[FFR_14,Power_14,A1_low_14,A1_high_14,A0_low_14,A0_high_14] = FFR_85_alt_aux(Load,SFOC,MAX_ENGINE_POWER_1); 
% evaluation of the SFOC minimum and of the angular coefficients for MEs 1 and 4
[FFR_23,Power_23,A1_low_23,A1_high_23,A0_low_23,A0_high_23] = FFR_85_alt_aux(Load,SFOC,MAX_ENGINE_POWER_2);
% evaluation of the SFOC minimum and of the angular coefficients for MEs 2 and 3

%the coefficients are the same
A1_low=[A1_low_14 A1_low_23 A1_low_23 A1_low_14]';
A1_high=[A1_high_14 A1_high_23 A1_high_23 A1_high_14]';
A0_low=[A0_low_14 A0_low_23 A0_low_23 A0_low_14]';
A0_high=[A0_high_14 A0_high_23 A0_high_23 A0_high_14]';

%for the matrix calculations
MAX_ENGINE_POWER=repmat([MAX_ENGINE_POWER_1 MAX_ENGINE_POWER_2 MAX_ENGINE_POWER_3 MAX_ENGINE_POWER_4],T,1);
MIN_ENGINE_POWER=repmat([MIN_ENGINE_POWER_14 MIN_ENGINE_POWER_23 MIN_ENGINE_POWER_23 MIN_ENGINE_POWER_14], T,1);

%%  Optimization variables

% engine On to define the interval of working
engOn_low = optimvar('engOn_low',T,4,'Type', 'integer','LowerBound',0,'UpperBound',1);
engOn_high = optimvar('engOn_high',T,4,'Type', 'integer','LowerBound',0,'UpperBound',1);

% low power and high power are separated
P_low = optimvar('P_low',T,4,'LowerBound',0);
P_high = optimvar('P_high',T,4,'LowerBound',0);

% for the turning on
TurnOn = optimvar('TurnOn',T-1,4,'Type','integer','LowerBound',0, 'UpperBound',1);

idx = 1:(T-1);
w = optimexpr(T-1,4);
w(idx,:) = engOn_low(idx+1,:) + engOn_high(idx+1,:) - (engOn_low(idx,:) + engOn_high(idx,:));

%% Objective function to minimize
fuelConsumptionObj = sum((engOn_low*A0_low+P_low*A1_low)+(engOn_high*A0_high+P_high*A1_high))...
    *TIME_STEP_DURATION + sum(sum((TurnOn*START_GRAMS))); % returns g/cicle

%% Constraints
% Power balance constraint
powerBalanceCons = sum(P_low + P_high, 2 ) >= Power_prop_demand_new;

% Constraint to force value of TurnOn variable
addGrams = w' - TurnOn' <= 0;

% constraints for the high power or low power
PowerLowMinCons = engOn_low.*MIN_ENGINE_POWER <= P_low;
PowerLowMaxCons = P_low <= 0.85*engOn_low.*MAX_ENGINE_POWER;
PowerHighMinCons = 0.85*engOn_high.*MAX_ENGINE_POWER <= P_high;
PowerHighMaxCons = P_high <= engOn_high.*MAX_ENGINE_POWER;

% constraints for engine On
engOnCons = engOn_low + engOn_high <= 1;

%% Problem setup
prob = optimproblem('ObjectiveSense', 'minimize');
prob.Objective = fuelConsumptionObj;
prob.Constraints.powerbalancecons = powerBalanceCons;

prob.Constraints.addGrams = addGrams;

%motors
prob.Constraints.PowerLowMinCons = PowerLowMinCons;
prob.Constraints.PowerLowMaxCons = PowerLowMaxCons;
prob.Constraints.PowerHighMinCons = PowerHighMinCons;
prob.Constraints.PowerHighMaxCons = PowerHighMaxCons;

%engOn
prob.Constraints.engOnCons=engOnCons;

options = optimoptions('intlinprog');
options.CutGeneration = 'advanced';
options.Heuristics = 'advanced';
options.BranchRule ='maxpscost';
options.MaxTime = time_sim;
options.IntegerPreprocess = 'advanced';

%% Solve
[probsol,fval,exitflag,output] = solve(prob,'Options',options);

%% Graphs

probsol.enginePower = probsol.P_low + probsol.P_high;
Power_engines_prop(:,:) = probsol.enginePower(:,:);

% Engine power1
figure()
subplot(2,2,1);
plot(t/3600, Power_engines_prop(:,1),'b')
xlabel('Time [h]')
ylabel('Engine Power 1 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

% Engine power2
subplot(2,2,2);
plot(t/3600, Power_engines_prop(:,2),'g')
xlabel('Time [h]')
ylabel('Engine Power 2 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

% Engine power3
subplot(2,2,3);
plot(t/3600, Power_engines_prop(:,3),'c')
xlabel('Time [h]')
ylabel('Engine Power 3 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

% Engine power4
subplot(2,2,4);
plot(t/3600, Power_engines_prop(:,4),'m')
xlabel('Time [h]')
ylabel('Engine Power 4 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

sgtitle('Engines Power')

% All plots
figure()
plot(t/3600, Power_prop_demand_new/1000,'r',t/3600, Power_engines_prop(:,1)/1000,'b',...
    t/3600, Power_engines_prop(:,2)/1000,'g',t/3600, Power_engines_prop(:,3)/1000,'c',...
    t/3600, Power_engines_prop(:,4)/1000,'m')
legend('Power demand','Engine power1','Engine power2','Engine power3','Engine power4')
ylabel('Power [MW]')
xlabel('Time [h]')
title ('Optimised propulsion engines')
xlim([0 t_end/3600])
grid on
grid minor

toc