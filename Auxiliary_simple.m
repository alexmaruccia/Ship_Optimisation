clc;
clear all;
close all;

time_sim = 300; %s for simulation
BATTERY_CAPACITY = 2032.34 *3600; % in kJ
C_rate_dis = 4;
C_rate_cha = 3;

MAX_ENGINE_POWER1 = 2400; % in kW
MAX_ENGINE_POWER2 = 3200; 
MAX_ENGINE_POWER3 = MAX_ENGINE_POWER2; 
MAX_ENGINE_POWER4 = MAX_ENGINE_POWER1; 
MIN_ENGINE_POWER14 = 0.2*MAX_ENGINE_POWER1; 
MIN_ENGINE_POWER23 = 0.2*MAX_ENGINE_POWER2; 

MAX_DISCHARGING_POWER =  BATTERY_CAPACITY*C_rate_dis; 
MAX_CHARGING_POWER =  BATTERY_CAPACITY*C_rate_cha; 
CHARGING_EFFICIENCY = 0.97;
DISCHARGING_EFFICIENCY = 0.98;
TIME_STEP_DURATION=120; %2 minutes
START_GRAMS = 2000;

% Power required
DataSheet = xlsread('Serenade auxiliary power');
powerDemand_aux=DataSheet(:,2);
t_aux=DataSheet(:,3)';
T=length(t_aux);
t_end=t_aux(end)/3600;

figure
plot(t_aux/3600,powerDemand_aux,'r')
xlabel('Time [h]')
ylabel('Auxiliary power demand [kW]')
title('Auxiliary power demand')
xlim([0 t_end])
grid on
grid minor

Consumption = xlsread('HFO_SFOC_AE'); %auxiliary engine
Load=Consumption(:,1);
SFOC=Consumption(:,2);

[FFR_14,Power_14,A1_low_14,A1_high_14,A0_low_14,A0_high_14] = FFR_85_alt_aux(Load,SFOC,MAX_ENGINE_POWER1); % evaluation of the SFOC minimum and of the angular coefficients for AE 1 and 4
[FFR_23,Power_23,A1_low_23,A1_high_23,A0_low_23,A0_high_23] = FFR_85_alt_aux(Load,SFOC,MAX_ENGINE_POWER2); % evaluation of the SFOC minimum and of the angular coefficients for AE 2 and 3

%the coefficients are the same
A1_low=[A1_low_14 A1_low_23 A1_low_23 A1_low_14]';
A1_high=[A1_high_14 A1_high_23 A1_high_23 A1_high_14]';
A0_low=[A0_low_14 A0_low_23 A0_low_23 A0_low_14]';
A0_high=[A0_high_14 A0_high_23 A0_high_23 A0_high_14]';

%for the matrix calculations
MAX_ENGINE_POWER=repmat([MAX_ENGINE_POWER1 MAX_ENGINE_POWER2 MAX_ENGINE_POWER3 MAX_ENGINE_POWER4],T,1);
MIN_ENGINE_POWER=repmat([MIN_ENGINE_POWER14 MIN_ENGINE_POWER23 MIN_ENGINE_POWER23 MIN_ENGINE_POWER14], T,1);

%%  Optimization variables

% engine On to define the interval of working
engOn_low = optimvar('engOn_low',T,4,'Type', 'integer','LowerBound',0,'UpperBound',1);
engOn_high = optimvar('engOn_high',T,4,'Type', 'integer','LowerBound',0,'UpperBound',1);


% low power and high power are separated
P_low = optimvar('P_low',T,4,'LowerBound',0);
P_high = optimvar('P_high',T,4,'LowerBound',0);

% for the battery
batteryPowerCh = optimvar('batteryPowerCh',T,1,'LowerBound',0);
batteryPowerDis = optimvar('batteryPowerDis',T,1,'LowerBound',0);
batterySoc = optimvar('batterySoc',T,'LowerBound',0.2,'UpperBound',0.95);
batterySoc0 = optimvar('batterySoc0','LowerBound',0.2,'UpperBound',0.95);

% for the turning on
TurnOn = optimvar('TurnOn',T-1,4,'Type','integer','LowerBound',0, 'UpperBound',1);

idx = 1:T-1;
w=optimexpr(T-1,4);
w(idx,:) = engOn_low(idx+1,:)+ engOn_high(idx+1,:)- (engOn_low(idx,:)+ engOn_high(idx,:));


%% Objective function to minimize
fuelConsumptionObj = sum((engOn_low*A0_low+P_low*A1_low)+(engOn_high*A0_high+P_high*A1_high))...
    *TIME_STEP_DURATION + sum(sum((TurnOn*START_GRAMS))); % returns g/cicle

%% Constraints
% Power balance constraint
powerBalanceCons = sum(P_low + P_high, 2 ) + batteryPowerDis*DISCHARGING_EFFICIENCY - batteryPowerCh/CHARGING_EFFICIENCY >= powerDemand_aux;

% Constraint to force value of TurnOn variable
addGrams = w' - TurnOn' <= 0;

newBatterySoc = optimexpr(T);
ii = 2:T;
newBatterySoc(1) = batterySoc0 + ((batteryPowerCh(1) - batteryPowerDis(1))/BATTERY_CAPACITY)*(TIME_STEP_DURATION);
newBatterySoc(ii) = batterySoc(ii-1) + (batteryPowerCh(ii) - batteryPowerDis(ii))/BATTERY_CAPACITY*(TIME_STEP_DURATION);

batteryBalanceCons = batterySoc == newBatterySoc;
cyclicBatteryCons = batterySoc0 == 0.95;
batteryLimit = batteryPowerDis <= powerDemand_aux;

% constraints for the high power or low power
PowerLowMinCons = engOn_low.*MIN_ENGINE_POWER <= P_low;
PowerLowMaxCons = P_low <= 0.85*engOn_low.*MAX_ENGINE_POWER;
PowerHighMinCons = 0.85*engOn_high.*MAX_ENGINE_POWER <= P_high;
PowerHighMaxCons = P_high <= engOn_high.*MAX_ENGINE_POWER;

% constraints for engine On
engOnCons = engOn_low + engOn_high <= 1;

% battery charge/discharge power
batteryMaxChPowerCons = batteryPowerCh <= MAX_CHARGING_POWER;
batteryMaxDisPowerCons = batteryPowerDis <= MAX_DISCHARGING_POWER;

%% Problem setup
prob = optimproblem('ObjectiveSense', 'minimize');
prob.Objective = fuelConsumptionObj;
prob.Constraints.powerbalancecons = powerBalanceCons;
prob.Constraints.addGrams = addGrams;

% engines
prob.Constraints.PowerLowMinCons = PowerLowMinCons;
prob.Constraints.PowerLowMaxCons = PowerLowMaxCons;
prob.Constraints.PowerHighMinCons = PowerHighMinCons;
prob.Constraints.PowerHighMaxCons = PowerHighMaxCons;

% engOn
prob.Constraints.engOnCons=engOnCons;

% battery
prob.Constraints.batteryMaxChPowerCons = batteryMaxChPowerCons;
prob.Constraints.batteryMaxDisPowerCons = batteryMaxDisPowerCons;
prob.Constraints.batteryBalanceCons = batteryBalanceCons;
prob.Constraints.cyclicBatteryCons = cyclicBatteryCons;
prob.Constraints.batteryLimit = batteryLimit;

options = optimoptions('intlinprog');
options.CutGeneration='advanced';
options.Heuristics='advanced';
options.BranchRule ='maxpscost';
options.MaxTime=time_sim;
options.IntegerPreprocess = 'advanced';

%% Solve
[probsol,fval,exitflag,output] = solve(prob,'solver','intlinprog','Options',options);

%% Graphs

probsol.enginePower = probsol.P_low + probsol.P_high;
Power_engines_aux(:,:) = probsol.enginePower(:,:);
Battery_trend = probsol.batteryPowerDis - probsol.batteryPowerCh;

% Power demand
figure
subplot(3,2,1);
plot(t_aux/3600, powerDemand_aux,'r')
% ylim([-100 100])
xlabel('Time [h]')
ylabel('Power demand [kW]')
xlim([0 t_end])
grid on
grid minor

% Engine power1
subplot(3,2,2);
plot(t_aux/3600, Power_engines_aux(:,1),'b')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power1 [kW]')
xlim([0 t_end])
grid on
grid minor

% Engine power2
subplot(3,2,3);
plot(t_aux/3600, Power_engines_aux(:,2),'g')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power2 [kW]')
xlim([0 t_end])
grid on
grid minor

% Engine power3
subplot(3,2,4);
plot(t_aux/3600, Power_engines_aux(:,3),'c')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power3 [kW]')
xlim([0 t_end])
grid on
grid minor

% Engine power4
subplot(3,2,5);
plot(t_aux/3600, Power_engines_aux(:,4),'m')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power4 [kW]')
xlim([0 t_end])
grid on
grid minor

% Battery power
subplot(3,2,6);
plot(t_aux/3600, Battery_trend,'k')
xlabel('Time [h]')
ylabel('Battery power [kW]')
xlim([0 t_end])
grid on
grid minor

sgtitle('Power Demand, AEs Power and Battery Power')

% All plots
figure
plot(t_aux/3600, powerDemand_aux,'r',t_aux/3600, Power_engines_aux(:,1),'b',...
    t_aux/3600, Power_engines_aux(:,2),'g',t_aux/3600, Power_engines_aux(:,3),'c',...
    t_aux/3600, Power_engines_aux(:,4),'m',t_aux/3600, Battery_trend,'k')
legend('Power demand','Engine power1','Engine power2','Engine power3','Engine power4','Battery power','Location','bestoutside')
ylabel('Power [kW]')
xlabel('Time [h]')
title('Optimised auxiliary motors and battery')
xlim([0 t_end])
grid on
grid minor

figure
plot(t_aux/3600, probsol.batterySoc)
ylabel('SOC')
xlabel('Time [h]')
title('Battery SOC')
xlim([0 t_end])
ylim([0 1])
grid on
grid minor

%% For the evaluated time

% evaluation for the given time data
idx_1 = [924:T];
idx_2 = [1:135];
idx = [idx_1 idx_2];
t_res = [t_aux(idx_1) t_aux(idx_2)+ t_aux(end)];
t_res = t_res - t_res(1);

% Power demand
figure
subplot(3,2,1);
plot(t_res/3600, powerDemand_aux(idx),'r')
xlabel('Time [h]')
ylabel('Power demand [kW]')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

% Engine power1
subplot(3,2,2);
plot(t_res/3600, Power_engines_aux(idx,1),'b')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power1 [kW]')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

% Engine power2
subplot(3,2,3);
plot(t_res/3600, Power_engines_aux(idx,2),'g')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power2 [kW]')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

% Engine power3
subplot(3,2,4);
plot(t_res/3600, Power_engines_aux(idx,3),'c')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power3 [kW]')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

% Engine power4
subplot(3,2,5);
plot(t_res/3600, Power_engines_aux(idx,4),'m')
ylim([0 3500])
xlabel('Time [h]')
ylabel('Engine Power4 [kW]')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

% Battery power
subplot(3,2,6);
plot(t_res/3600, Battery_trend(idx),'k')
xlabel('Time [h]')
ylabel('Battery power [kW]')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

sgtitle('Power Demand, Engine Power and Battery Power for the evaluated time')

% All plots
figure
plot(t_res/3600, powerDemand_aux(idx),'r',t_res/3600, Power_engines_aux(idx,1),'b',...
    t_res/3600, Power_engines_aux(idx,2),'g',t_res/3600, Power_engines_aux(idx,3),'c',...
    t_res/3600, Power_engines_aux(idx,4),'m',t_res/3600, Battery_trend(idx),'k')
legend('Power demand','Engine power1','Engine power2','Engine power3','Engine power4','Battery power','Location','bestoutside')
ylabel('Power [kW]')
xlabel('Time [h]')
title('Optimised auxiliary motors and battery for the evaluated time')
xlim([t_res(1) t_res(end)]/3600)
grid on
grid minor

figure
plot(t_res/3600, probsol.batterySoc(idx))
ylabel('SOC')
xlabel('Time [h]')
title('Battery SOC for the evaluated time')
xlim([t_res(1) t_res(end)]/3600)
ylim([0 1])
grid on
grid minor

%% Evaluation of the restricted fuel consumption
FC_restricted = sum(probsol.engOn_low(idx,:)*A0_low+probsol.P_low(idx,:)*A1_low+...
    (probsol.engOn_high(idx,:)*A0_high+probsol.P_high(idx,:)*A1_high))...
    *TIME_STEP_DURATION + sum(sum(probsol.TurnOn(idx(idx<=1440),:)*START_GRAMS)) % returns g/cicle

fprintf ('The fuel consumption in the considered time is %.2f kg', FC_restricted/10^3)
