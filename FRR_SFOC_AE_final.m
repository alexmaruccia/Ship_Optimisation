% Load [%/100]
% SFC [g/kWh]
clc
clear all
close all

Consumption = xlsread('HFO_SFOC_AE');
Load_in=Consumption(:,1);
SFOC_in=Consumption(:,2);
%SFOC_in=[192.3 182.7 182.2 183.3]; %AE
P_max_1=2400;
P_max_2=3200;

%% row data
figure
plot(Load_in*100,SFOC_in)
xlabel('Load [%]') 
ylabel('SFOC [g/kWh]') 
title('SFOC AE from data')
xlim([50 100])
grid on

figure 
Power_1=P_max_1*Load_in;
FFR_1=Power_1.*SFOC_in/3600;
plot(Power_1,FFR_1)
xlim ([Power_1(1) Power_1(end)])
xlabel('Power [kW]') 
ylabel('FFR [g/s]') 
title('Fuel flow rate from data - AE, P max 2400 kW')
grid on
figure 

Power_2=P_max_2*Load_in;
FFR_2=Power_2.*SFOC_in/3600;
plot(Power_2,FFR_2)
xlim ([Power_2(1) Power_2(end)])
xlabel('Power [kW]') 
ylabel('FFR [g/s]') 
title('Fuel flow rate from data - AE, P max 3200 kW')
grid on

%% interpolated data 2400 kW

% 3rd deg polynomial
p3 = polyfit(Load_in,SFOC_in,3);
Load_3 = linspace(0,1);
SFOC_3 = polyval(p3,Load_3);
Power_13=P_max_1*Load_3;
FFR_13=Power_13.*SFOC_3/3600;


% 0 - 85 %
p1 = polyfit(Power_13(1:85),FFR_13(1:85),1);
Power_first = linspace(0,P_max_1); %kW
FFR_first = polyval(p1,Power_first); %g/s

SFOC_first = FFR_first*3600./Power_first; %g/MWh
Load_first=linspace(0,1); %Load from 0 to 1

% 85 - 100 %
p2 = polyfit(Power_13(86:100),FFR_13(86:100),1);
Power_second = linspace(0,P_max_1);
FFR_second = polyval(p2,Power_second);

SFOC_second = FFR_second*3600./Power_second;
Load_second=linspace(0,1);

% 3rd deg polynomial
p3 = polyfit(Load_in,SFOC_in,3);
Load_3 = linspace(0,1);
SFOC_3 = polyval(p3,Load_3);

figure('pos',[10 50 600 400]);
subplot(1,2,1);
plot(Power_1,FFR_1,'o','MarkerSize',10,'LineWidth',2,'Color','r'); %real point
grid;
hold on;
plot(Power_first(1:85),FFR_first(1:85), 'LineWidth', 2.5, 'Color','b'); %0-85% streight line
plot(Power_first,FFR_first, '--','LineWidth', 1.5, 'Color','b'); %continuing of the streight line
plot(Power_second(85:100),FFR_second(85:100), 'LineWidth', 2.5, 'Color','b');%86-100% streight line
plot(Power_second,FFR_second, '--','LineWidth', 1.5, 'Color','b'); 
legend('Project Guide data projection', 'Two-piece linear fit', 'Location', 'northeast');
xlim([0 P_max_1]);
ylim([0 180]);
xlabel('Power [kW]');
ylabel('Fuel flow rate [g/s]');
title('Interpolated FFR for AE, P max 2400 kW')

subplot(1,2,2);
plot(Load_in*100,SFOC_in,'o','MarkerSize',10,'LineWidth',2,'Color','r');
hold on
plot(Load_3*100,SFOC_3,'LineWidth', 2.5, 'Color','g');
hold on
plot(Power_first(1:85)/P_max_1*100, SFOC_first(1:85), 'LineWidth', 2.5, 'Color','b');
hold on
plot(Power_second(85:100)/P_max_1*100, SFOC_second(85:100), 'LineWidth', 2.5, 'Color','b');
legend('Project Guide data','3rd degree polynomial fit', 'Fuel flow rate linear fit projection', 'Location', 'northwest');
xlabel('Load [%]');
ylabel('Specific fuel consumption [g/kWh]');
xlim([0 100]);
ylim([175 270]);
grid on
title ('Interpolated SFOC for AE')


figure('pos',[1000 50 500 400]);
plot(Load_in*100,SFOC_in,'o','MarkerSize',10,'LineWidth',2,'Color','r');
hold on
plot(Power_first(1:85)/P_max_1*100,  SFOC_first(1:85), '--','LineWidth', 1.5, 'Color','b')
plot(Power_second(85:100)/P_max_1*100, SFOC_second(85:100), '--','LineWidth', 1.5, 'Color','b');
ylim ([170 280])
legend('Specific fuel consuption', 'Location', 'northeast');
xlabel('Load [%]');
ylabel('Specific fuel consumption [g/kWh]');
grid on
title ('Interpolated SFOC for AE')
%%

% interpolated values for FRR and Power
FFR_out=[FFR_first(1:85),FFR_second(86:100)];
Power_out=[Power_first(1:85) Power_second(86:100)];
% angular and constant coefficient
A_low=p1(1); %angular coefficient of the low part
A0_low=p1(2); % intersection with P=0;
A_high=p2(1); %angular coefficient of the high part
A0_high=p2(2); % intersection with P=0;
%interpolated values for SFOC and Load
SFOC_out=[SFOC_first(1:85), SFOC_second(86:100)];
Load_out= [Load_first(1:85) Load_second(86:100)];

%% interpolated data 3200 kW

Power_2 = P_max_2*Load_in; % Power output [kW]
FFR_2 = Power_2.*(SFOC_in/3600); % Fuel flow rate [g/s]

% 3rd deg polynomial
p3 = polyfit(Load_in,SFOC_in,3);
Load_3 = linspace(0,1);
SFOC_3 = polyval(p3,Load_3);
Power_23 = P_max_2*Load_3; % Power output [kW]
FFR_23 = Power_23.*(SFOC_3/3600); % Fuel flow rate [g/s]

% 0 - 85 %
p1 = polyfit(Power_23(1:85),FFR_23(1:85),1);
Power_first = linspace(0,P_max_2); %kW
FFR_first = polyval(p1,Power_first); %g/s

SFOC_first = FFR_first*3600./Power_first; %g/MWh
Load_first=linspace(0,1); %Load from 0 to 1

% 85 - 100 %
p2 = polyfit(Power_23(86:100),FFR_23(86:100),1);
Power_second = linspace(0,P_max_2);
FFR_second = polyval(p2,Power_second);

SFOC_second = FFR_second*3600./Power_second;
Load_second=linspace(0,1);

% 3rd deg polynomial
p3 = polyfit(Load_in,SFOC_in,3);
Load_3 = linspace(0,1);
SFOC_3 = polyval(p3,Load_3);

figure('pos',[10 50 600 400]);
subplot(1,2,1);
plot(Power_2,FFR_2,'o','MarkerSize',10,'LineWidth',2,'Color','r'); %real point
grid;
hold on;
plot(Power_first(1:85),FFR_first(1:85), 'LineWidth', 2.5, 'Color','b'); %0-85% streight line
plot(Power_first,FFR_first, '--','LineWidth', 1.5, 'Color','b'); %continuing of the streight line
plot(Power_second(85:100),FFR_second(85:100), 'LineWidth', 2.5, 'Color','b');%86-100% streight line
plot(Power_second,FFR_second, '--','LineWidth', 1.5, 'Color','b'); 
legend('Project Guide data projection', 'Two-piece linear fit', 'Location', 'northeast');
xlim([0 P_max_2]);
ylim([0 180]);
xlabel('Power [kW]');
ylabel('Fuel flow rate [g/s]');
title('Interpolated FFR for AE, P max 3200 kW')

subplot(1,2,2);
plot(Load_in*100,SFOC_in,'o','MarkerSize',10,'LineWidth',2,'Color','r');
hold on
plot(Load_3*100,SFOC_3,'LineWidth', 2.5, 'Color','g');
hold on
plot(Power_first(1:85)/P_max_2*100, SFOC_first(1:85), 'LineWidth', 2.5, 'Color','b');
hold on
plot(Power_second(85:100)/P_max_2*100, SFOC_second(85:100), 'LineWidth', 2.5, 'Color','b');
legend('Project Guide data','3rd degree polynomial fit', 'Fuel flow rate linear fit projection', 'Location', 'northwest');
xlabel('Load [%]');
ylabel('Specific fuel consumption [g/kWh]');
xlim([0 100]);
ylim([175 270]);
grid on
title ('Interpolated SFOC for AE')

figure('pos',[1000 50 500 400]);
plot(Load_in*100,SFOC_in,'o','MarkerSize',10,'LineWidth',2,'Color','r');
hold on
plot(Power_first(1:85)/P_max_2*100,  SFOC_first(1:85), '--','LineWidth', 1.5, 'Color','b')
plot(Power_second(85:100)/P_max_2*100, SFOC_second(85:100), '--','LineWidth', 1.5, 'Color','b');
legend('Specific fuel consuption', 'Location', 'northeast');
ylim ([170 280])
xlabel('Load [%]');
ylabel('Specific fuel consumption [g/kWh]');
grid on
title ('Interpolated SFOC for AE')


