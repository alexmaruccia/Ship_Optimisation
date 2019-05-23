clc
clear all
%close all

DataSheet = xlsread('Serenade AIS and ME 2018-12-16','Sensible Data');
Prop_power=DataSheet(:,3); %MW
Prop_AIS_speed=DataSheet(:,4); %kn
Prop_wind_speed=DataSheet(:,5); %m/s
Prop_wind_direction=DataSheet(:,6); %deg
Prop_time=DataSheet(:,2); %s
Prop_time_end=Prop_time(end);

% plot(Prop_time)
subplot(2,1,1)
plot(Prop_time/3600,Prop_power/1000)
xlabel('Time [h]') 
ylabel('Propulsion power demand [MW]') 
title('Propulsion power demand')
xlim([0 Prop_time_end/3600])
grid on

DataSheet = xlsread('Serenade auxiliary power');
powerDemand=DataSheet(:,2)*1;
t=DataSheet(:,3)';
T=length(t);
t_end=t(end)/3600;

subplot(2,1,2)
plot(t/3600,powerDemand/1000,'r')
xlabel('Time [h]') 
ylabel('Power demand [MW]') 
title('Auxiliary power demand')
xlim([0 t_end])
grid on

figure(2)
yyaxis left
plot(Prop_time/3600,Prop_power/1000,'b')
xlabel('Time [h]') 
ylabel('Propulsion power demand [MW]') 
title('Propulsion power demand and ship speed')
xlim([0 Prop_time_end/3600])
grid on
hold on
yyaxis right
plot(Prop_time/3600,Prop_AIS_speed,'r')
ylabel('Ship speed [kn]') 

figure(3)
yyaxis left
plot(Prop_time/3600,Prop_wind_speed,'b')
xlabel('Time [h]') 
ylabel('Wind speed [m/s]') 
title('Wind values')
xlim([0 Prop_time_end/3600])
grid on
hold on
yyaxis right
plot(Prop_time/3600,Prop_wind_direction,'r')
ylabel('Wind direction [deg]') 


