clc
clear all
close all

%% Evaluation of the FR contribution

% data of time from data (only time steps are needed)
[Total_power, Power_prop_demand, Power_aux_demand, t, T, t_end] = Total_power_evaluation_func_adv();

% sin angular velocity
w =  2*pi/t_end ;

% maximum wind speed = 20 m/s; min = 5 m/s
max_wind = 25 ;
min_wind = 10 ;
Y0 = (max_wind + min_wind)/2; % medium value
A0 = 2*(max_wind - Y0); % Amplitude

wind = A0/2*cos(w*t - 140) + Y0;
angle = 180*cos(w*t - 60) + 180; 

% plot wind and angle
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
title ('True wind speed and Relative angle')

% evaluation of the interpoleted values and the resulting thrust: creation of the interpoleted points with the lookup tables
[wind_val, angle_val, Thrust_int] = Look_up_tables_polar_diagram_func();

wind = round (wind);
angle = round (angle);

%indices of wind
idx_wind = zeros (length(wind),1);

for i= 1:length(idx_wind)
    idx_wind(i) = find (wind_val == wind(i));
end

idx_angle = zeros (length(angle),1);

for i= 1:length(angle)
    idx_angle(i) = find (angle_val == angle(i));
end

idx = sub2ind( size(Thrust_int), idx_wind, idx_angle); % creation of coordinates for evaluating the thrust

Thrust = Thrust_int(idx); % evaluation of the thrust supplied (as power, in kW)

% Considering a power decreasing of 20% when the ship's speed is about 15 knots
Thrust([1:find(t == 8940)]) = Thrust([1:find(t == 8940)])*0.8;

% Considering a power decreasing of 50% when the ship's speed is zero
Thrust(find(t == 56220):en]) = Thrust(find(t == 56220):end)*0.5;

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

time_sim = 600;
START_GRAMS_PROP = 3500;
START_GRAMS_AUX = 2000;
TIME_STEP_DURATION = t(2) - t(1);

% battery data
BATTERY_CAPACITY = 2032.34 *3600; % in kJ
C_rate_dis = 4;
C_rate_cha = 3;
MAX_DISCHARGING_POWER =  BATTERY_CAPACITY*C_rate_dis; % in kW
MAX_CHARGING_POWER =  BATTERY_CAPACITY*C_rate_cha; 
CHARGING_EFFICIENCY = 0.97;
DISCHARGING_EFFICIENCY = 0.98;

% MEs
MAX_ME_POWER_1 = 8125; % in kW
MAX_ME_POWER_2 = 7500;
MAX_ME_POWER_3 = 7500;
MAX_ME_POWER_4 = 8125;

MIN_ME_POWER_14 = 0.2*MAX_ME_POWER_1; 
MIN_ME_POWER_23 = 0.2*MAX_ME_POWER_2;

% matrixes for MEs
MAX_ME_POWER=repmat([MAX_ME_POWER_1 MAX_ME_POWER_2 MAX_ME_POWER_3 MAX_ME_POWER_4],T,1);
MIN_ME_POWER=repmat([MIN_ME_POWER_14 MIN_ME_POWER_23 MIN_ME_POWER_23 MIN_ME_POWER_14], T,1);

% AEs
MAX_AE_POWER1 = 2400; % in kW 
MAX_AE_POWER2 = 3200; 
MAX_AE_POWER3 = 3200; 
MAX_AE_POWER4 = 2400; 

MIN_AE_POWER14 = 0.2*MAX_AE_POWER1; 
MIN_AE_POWER23 = 0.2*MAX_AE_POWER2; 

% matrixes for AEs
MAX_AE_POWER = repmat([MAX_AE_POWER1 MAX_AE_POWER2 MAX_AE_POWER3 MAX_AE_POWER4],T,1);
MIN_AE_POWER = repmat([MIN_AE_POWER14 MIN_AE_POWER23 MIN_AE_POWER23 MIN_AE_POWER14], T,1);

%shaft
MAX_SHAFT_POWER = 3500;
SHAFT_EFFICIENCY = 0.85;

% evaluation of the coeffiecients: ME

Consumption_ME = xlsread('HFO_SFOC_ME');
Load_ME = Consumption_ME(:,1);
SFOC_ME = Consumption_ME(:,2);

[FFR_14_ME,Power_14_ME,A1_low_14_ME,A1_high_14_ME,A0_low_14_ME,A0_high_14_ME] = FFR_85_alt(Load_ME,SFOC_ME,MAX_ME_POWER_1); 
% evaluation of the SFOC minimum and of the angular coefficients for MEs 1 and 4
[FFR_23_ME,Power_23_ME,A1_low_23_ME,A1_high_23_ME,A0_low_23_ME,A0_high_23_ME] = FFR_85_alt(Load_ME,SFOC_ME,MAX_ME_POWER_2); 
% evaluation of the SFOC minimum and of the angular coefficients for MEs 2 and 3

A1_low_ME = [A1_low_14_ME A1_low_23_ME A1_low_23_ME A1_low_14_ME]';
A1_high_ME = [A1_high_14_ME A1_high_23_ME A1_high_23_ME A1_high_14_ME]';
A0_low_ME = [A0_low_14_ME A0_low_23_ME A0_low_23_ME A0_low_14_ME]';
A0_high_ME = [A0_high_14_ME A0_high_23_ME A0_high_23_ME A0_high_14_ME]';

% evaluation of the coeffiecients: AE

Consumption_AE = xlsread('HFO_SFOC_AE'); %auxiliary engine
Load_AE = Consumption_AE(:,1);
SFOC_AE = Consumption_AE(:,2);

% evaluation of the SFOC minimum and of the angular coefficients for AEs 1 and 4
[FFR_14_AE,Power_14_AE,A1_low_14_AE,A1_high_14_AE,A0_low_14_AE,A0_high_14_AE] = FFR_85_alt(Load_AE,SFOC_AE,MAX_AE_POWER1);
% evaluation of the SFOC minimum and of the angular coefficients for AEs 2 and 3
[FFR_23_AE,Power_23_AE,A1_low_23_AE,A1_high_23_AE,A0_low_23_AE,A0_high_23_AE] = FFR_85_alt(Load_AE,SFOC_AE,MAX_AE_POWER2); 

A1_low_AE = [A1_low_14_AE A1_low_23_AE A1_low_23_AE A1_low_14_AE]';
A1_high_AE = [A1_high_14_AE A1_high_23_AE A1_high_23_AE A1_high_14_AE]';
A0_low_AE = [A0_low_14_AE A0_low_23_AE A0_low_23_AE A0_low_14_AE]';
A0_high_AE = [A0_high_14_AE A0_high_23_AE A0_high_23_AE A0_high_14_AE]';

% matrix assemble; MEs first, then AEs
MAX_ENGINE_POWER = [MAX_ME_POWER MAX_AE_POWER];
MIN_ENGINE_POWER = [MIN_ME_POWER MIN_AE_POWER];

A1_low = [A1_low_ME; A1_low_AE];
A1_high = [A1_high_ME; A1_high_AE];
A0_low = [A0_low_ME; A0_low_AE];
A0_high = [A0_high_ME; A0_high_AE];

% matric of starting masses
START_GRAMS = [repmat(START_GRAMS_PROP, T-1,4) repmat(START_GRAMS_AUX, T-1,4)];

%%  Optimization variables

% engine On to define the interval of working
engOn_low = optimvar('engOn_low',T,8,'Type', 'integer','LowerBound',0,'UpperBound',1);
engOn_high = optimvar('engOn_high',T,8,'Type', 'integer','LowerBound',0,'UpperBound',1);


% low power and high power are separated
P_low = optimvar('P_low',T,8,'LowerBound',0);
P_high = optimvar('P_high',T,8,'LowerBound',0);

% Shaft power
P_shaft = optimvar('P_shaft',T,'LowerBound',0);

%for the turning on
TurnOn = optimvar('TurnOn',T-1,8,'Type','integer','LowerBound',0, 'UpperBound',1);

idx = 1:(T-1);
w=optimexpr(T-1,8);
w(idx,:) = engOn_low(idx+1,:)+ engOn_high(idx+1,:)- (engOn_low(idx,:)+ engOn_high(idx,:));

% for the battery
batteryPowerCh = optimvar('batteryPowerCh',T,1,'LowerBound',0);
batteryPowerDis = optimvar('batteryPowerDis',T,1,'LowerBound',0);
batterySoc = optimvar('batterySoc',T,'LowerBound',0.20,'UpperBound',0.95);
batterySoc0 = optimvar('batterySoc0','LowerBound',0.20,'UpperBound',0.95);

% Objective function to minimize
fuelConsumptionObj = sum((engOn_low*A0_low+P_low*A1_low)+(engOn_high*A0_high+P_high*A1_high))...
    *TIME_STEP_DURATION + sum(sum((TurnOn.*START_GRAMS)))*TIME_STEP_DURATION; % returns g/cicle

%% Constraints

% ME Power balance constraint
powerBalanceCons_ME = sum(P_low (:,[1:4]) + P_high (:,[1:4]), 2) - P_shaft  >= Power_prop_demand_new;

% AE Power balance constraint
powerBalanceCons_AE = sum(P_low (:,[5:8]) + P_high (:,[5:8]), 2 ) + batteryPowerDis*DISCHARGING_EFFICIENCY - batteryPowerCh/CHARGING_EFFICIENCY + P_shaft*SHAFT_EFFICIENCY >= Power_aux_demand;

% Constraint to force value of TurnOn variable
addGrams = w' - TurnOn' <= 0;

% constraints for the high power or low power
PowerLowMinCons = engOn_low.*MIN_ENGINE_POWER <= P_low;
PowerLowMaxCons = P_low <= 0.85*engOn_low.*MAX_ENGINE_POWER;
PowerHighMinCons = 0.85*engOn_high.*MAX_ENGINE_POWER <= P_high;
PowerHighMaxCons = P_high <= engOn_high.*MAX_ENGINE_POWER;

% Constraints for shaft generator
MaxShaftPower = P_shaft <= MAX_SHAFT_POWER;

newBatterySoc = optimexpr(T);
ii = 2:T;
newBatterySoc(1) = batterySoc0 + ((batteryPowerCh(1) - batteryPowerDis(1))/BATTERY_CAPACITY)*(TIME_STEP_DURATION);
newBatterySoc(ii) = batterySoc(ii-1) + (batteryPowerCh(ii) - batteryPowerDis(ii))/BATTERY_CAPACITY*(TIME_STEP_DURATION);

batteryBalanceCons = batterySoc == newBatterySoc;
initialBatteryCons = batterySoc0 == 0.95;
batteryLimit = batteryPowerDis <= Power_aux_demand;

% constraints for engine On
engOnCons = engOn_low + engOn_high <= 1;

%% Problem setup
prob = optimproblem('ObjectiveSense', 'minimize');
prob.Objective.Prop = fuelConsumptionObj;

prob.Constraints.powerBalanceCons_ME = powerBalanceCons_ME;
prob.Constraints.powerBalanceCons_AE = powerBalanceCons_AE;
prob.Constraints.addGrams = addGrams;

%motors
prob.Constraints.PowerLowMinCons = PowerLowMinCons;
prob.Constraints.PowerLowMaxCons = PowerLowMaxCons;
prob.Constraints.PowerHighMinCons = PowerHighMinCons;
prob.Constraints.PowerHighMaxCons = PowerHighMaxCons;

prob.Constraints.MaxShaftPower = MaxShaftPower;

%engOn
prob.Constraints.engOnCons=engOnCons;

prob.Constraints.batteryBalanceCons = batteryBalanceCons;
prob.Constraints.initialBatteryCons = initialBatteryCons;
prob.Constraints.batteryLimit = batteryLimit;

% options for the simulation
options = optimoptions('intlinprog');
options.CutGeneration = 'advanced';
options.Heuristics = 'advanced';
options.BranchRule ='maxpscost';
options.MaxTime = time_sim;
options.IntegerPreprocess = 'advanced';

%% Solve
[probsol,fval,exitflag,output] = solve(prob,'Options',options);

%% Graphs

probsol.enginePower = probsol.P_low+probsol.P_high;
Power_engines_prop(:,:) = probsol.enginePower(:,[1:4]);
Power_engines_aux(:,:) = probsol.enginePower(:,[5:8]);
Battery_trend = probsol.batteryPowerDis - probsol.batteryPowerCh;
Power_shaft_converted = SHAFT_EFFICIENCY * probsol.P_shaft;

% ME power1
figure()
subplot(2,2,1);
plot(t/3600, Power_engines_prop(:,1),'b')
xlabel('Time [h]')
ylabel('Engine Power 1 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

% ME power2
subplot(2,2,2);
plot(t/3600, Power_engines_prop(:,2),'g')
xlabel('Time [h]')
ylabel('Engine Power 2 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

% ME power3
subplot(2,2,3);
plot(t/3600, Power_engines_prop(:,3),'c')
xlabel('Time [h]')
ylabel('Engine Power 3 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

% ME power4
subplot(2,2,4);
plot(t/3600, Power_engines_prop(:,4),'m')
xlabel('Time [h]')
ylabel('Engine Power 4 [kW]')
xlim([0 t_end/3600])
ylim ([0 8125])
grid on
grid minor

sgtitle('MEs Power')

% ME plots
figure()
plot(t/3600, Power_prop_demand/1000,'r',t/3600, Power_engines_prop(:,1)/1000,'b',...
    t/3600, Power_engines_prop(:,2)/1000,'g',t/3600, Power_engines_prop(:,3)/1000,'c',...
    t/3600, Power_engines_prop(:,4)/1000,'m')
legend('Power demand','Engine power1','Engine power2','Engine power3','Engine power4')
ylabel('Power [MW]')
xlabel('Time [h]')
title('Optimised MEs with shaft generator and FR')
xlim([0 t_end/3600])
grid on
grid minor

% AE power demand
figure
subplot(3,2,1);
plot(t/3600, Power_aux_demand - Power_shaft_converted,'r')
xlabel('Time [h]')
ylabel('Power demand [kW]')
xlim([0 t_end/3600])
grid on
grid minor

% AE power1
subplot(3,2,2);
plot(t/3600, Power_engines_aux(:,1),'b')
ylim([0 3500])
xlabel('Time [h]')
ylabel('AE 1 Power [kW]')
xlim([0 t_end/3600])
grid on
grid minor

% AE power2
subplot(3,2,3);
plot(t/3600, Power_engines_aux(:,2),'g')
ylim([0 3500])
xlabel('Time [h]')
ylabel('AE 2 Power [kW]')
xlim([0 t_end/3600])
grid on
grid minor

% AE power3
subplot(3,2,4);
plot(t/3600, Power_engines_aux(:,3),'c')
ylim([0 3500])
xlabel('Time [h]')
ylabel('AE 3 Power [kW]')
xlim([0 t_end/3600])
grid on
grid minor

% AE power4
subplot(3,2,5);
plot(t/3600, Power_engines_aux(:,4),'m')
ylim([0 3500])
xlabel('Time [h]')
ylabel('AE 4 Power [kW]')
xlim([0 t_end/3600])
grid on
grid minor

% AE power
subplot(3,2,6);
plot(t/3600, Battery_trend,'k')
xlabel('Time [h]')
ylabel('Battery power [kW]')
xlim([0 t_end/3600])
grid on
grid minor

sgtitle('Power Demand, AEs Power and Battery Power')

% AE plots
figure
plot(t/3600, Power_aux_demand - Power_shaft_converted,'r',t/3600, Power_engines_aux(:,1),'b',...
    t/3600, Power_engines_aux(:,2),'g',t/3600, Power_engines_aux(:,3),'c',...
    t/3600, Power_engines_aux(:,4),'m',t/3600, Battery_trend,'k')
legend('Power demand','AE 1','AE 2','AE 3','AE 4','Battery power','Location','bestoutside')
ylabel('Power [kW]')
xlabel('Time [h]')
title('Optimised AEs and battery with shaft generator and FR')
xlim([0 t_end/3600])
grid on
grid minor

% plot of the shaft
figure()
plot(t/3600, probsol.P_shaft,'g')
xlabel('Time [h]')
ylabel('Shaft power [kW]')
xlim([0 t_end/3600])
ylim ([0 3500])
title ('Shaft converted power')
grid on
grid minor

figure
plot(t/3600, probsol.batterySoc)
ylabel('SOC')
xlabel('Time [h]')
title('Battery SOC')
xlim([0 t_end/3600])
ylim([0 1])
grid on
grid minor

figure
plot (t/3600, Power_aux_demand, 'r')
hold on
plot (t/3600, Battery_trend, 'k')
plot (t/3600, Power_shaft_converted ,'g')
legend ('Auxiliary Power Demand', 'Battery trend', 'Shaft generator power transmitted')
xlim([0 t_end/3600])
grid on
grid minor
