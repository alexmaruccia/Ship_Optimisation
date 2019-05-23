
clc;
clear all; 
close all;
%%
time_sim = 300; % s for simulation
tic
cd 'C:\Users\Alessandro\Desktop\THESIS\Model'

MAX_ENGINE_POWER_1 = 8125; % in kW
MAX_ENGINE_POWER_2 = 7500;
MAX_ENGINE_POWER_3 = 7500;
MAX_ENGINE_POWER_4 = 8125;

MIN_ENGINE_POWER_14 = 0.2*MAX_ENGINE_POWER_1; 
MIN_ENGINE_POWER_23 = 0.2*MAX_ENGINE_POWER_2;

TIME_STEP_DURATION = 600; %5 minutes
START_GRAMS = 3500;

% Power required
DataSheet = xlsread('Serenade AIS and ME 2018-12-16','Sensible Data');
powerDemand_prop = DataSheet(:,3); % in kW
t_prop = DataSheet(:,2); %s
T = length(t_prop);
t_end = t_prop(end)/3600;

figure
plot(t_prop/3600,powerDemand_prop/1000,'r')
xlabel('Time [h]') 
ylabel('Propulsion power demand [MW]') 
title('Propulsion power demand')
xlim([0 t_end])
grid on
grid minor

Consumption = xlsread('HFO_SFOC_ME');
Load=Consumption(:,1);
SFOC=Consumption(:,2);

[FFR_14,Power_14,A1_low_14,A1_high_14,A0_low_14,A0_high_14] = FFR_85_alt_aux(Load,SFOC,MAX_ENGINE_POWER_1); % evaluation of the SFOC minimum and of the angular coefficients for MEs 1 and 4
[FFR_23,Power_23,A1_low_23,A1_high_23,A0_low_23,A0_high_23] = FFR_85_alt_aux(Load,SFOC,MAX_ENGINE_POWER_2); % evaluation of the SFOC minimum and of the angular coefficients for MEs 2 and 3

% arrays of coeffiecients
A1_low=[A1_low_14 A1_low_23 A1_low_23 A1_low_14]';
A1_high=[A1_high_14 A1_high_23 A1_high_23 A1_high_14]';
A0_low=[A0_low_14 A0_low_23 A0_low_23 A0_low_14]';
A0_high=[A0_high_14 A0_high_23 A0_high_23 A0_high_14]';

% for the matrix calculations
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
w=optimexpr(T-1,4);
w(idx,:) = engOn_low(idx+1,:)+ engOn_high(idx+1,:) - (engOn_low(idx,:)+ engOn_high(idx,:));

%% Objective function to minimize
fuelConsumptionObj = sum((engOn_low*A0_low+P_low*A1_low)+(engOn_high*A0_high+P_high*A1_high))...
    *TIME_STEP_DURATION + sum(sum((TurnOn*START_GRAMS))); % returns g/cicle

%% Constraints
% Power balance constraint
powerBalanceCons = sum(P_low + P_high, 2) >= powerDemand_prop;

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

% engines
prob.Constraints.PowerLowMinCons = PowerLowMinCons;
prob.Constraints.PowerLowMaxCons = PowerLowMaxCons;
prob.Constraints.PowerHighMinCons = PowerHighMinCons;
prob.Constraints.PowerHighMaxCons = PowerHighMaxCons;

% engOn
prob.Constraints.engOnCons=engOnCons;

%options fo the simulation
options = optimoptions('intlinprog');
options.CutGeneration = 'advanced';
options.Heuristics = 'advanced';
options.BranchRule ='strongpscost';
 options.MaxTime = time_sim;
options.HeuristicsMaxNodes = 200;
options.IntegerPreprocess = 'advanced';

%% Solve
[probsol,fval,exitflag,output] = solve(prob,'Options',options);

%% Graphs
probsol.enginePower = probsol.P_low + probsol.P_high; % global engine power
Power_engines_prop(:,:) = probsol.enginePower(:,:);

% Engine power1
figure()
subplot(2,2,1);
plot(t_prop/3600, Power_engines_prop(:,1),'b')
xlabel('Time [h]') 
ylabel('Engine Power 1 [kW]') 
xlim([0 t_end])
ylim ([0 8125])
grid on
grid minor

% Engine power2
subplot(2,2,2);
plot(t_prop/3600, Power_engines_prop(:,2),'g')
xlabel('Time [h]') 
ylabel('Engine Power 2 [kW]') 
xlim([0 t_end])
ylim ([0 8125])
grid on
grid minor

% Engine power3
subplot(2,2,3);
plot(t_prop/3600, Power_engines_prop(:,3),'c')
% ylim([0 80])
xlabel('Time [h]') 
ylabel('Engine Power 3 [kW]') 
xlim([0 t_end])
ylim ([0 8125])
grid on
grid minor

% Engine power4
subplot(2,2,4);
plot(t_prop/3600, Power_engines_prop(:,4),'m')
xlabel('Time [h]') 
ylabel('Engine Power 4 [kW]') 
xlim([0 t_end])
ylim ([0 8125])
grid on
grid minor

sgtitle('Engines Power')

% All plots
figure()
plot(t_prop/3600, powerDemand_prop/1000,'r',t_prop/3600, Power_engines_prop(:,1)/1000,'b',...
    t_prop/3600, Power_engines_prop(:,2)/1000,'g',t_prop/3600, Power_engines_prop(:,3)/1000,'c',...
    t_prop/3600, Power_engines_prop(:,4)/1000,'m')
legend('Power demand','Engine power1','Engine power2','Engine power3','Engine power4')
ylabel('Power [MW]')
xlabel('Time [h]')
title ('Optimised propulsion engines')
xlim([0 t_end])
grid on
grid minor

toc