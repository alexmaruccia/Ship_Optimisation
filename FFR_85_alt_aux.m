function [FFR_out,Power_out,A_low,A_high,A0_low,A0_high,SFOC_out,Load_out] = FFR_85(Load_in,SFOC_in,P_max)

% Load [%/100]
% SFC [g/kWh]

Power = P_max*Load_in; % Power output [kW]
FFR = Power.*(SFOC_in/3600); % Fuel flow rate [g/s]

% 3rd deg polynomial
p3 = polyfit(Load_in,SFOC_in,3);
Load_3 = linspace(0,1);
SFOC_3 = polyval(p3,Load_3);

Power_3 = P_max*Load_3; % Power output [kW]
FFR_3 = Power_3.*(SFOC_3/3600); % Fuel flow rate [g/s]

% 0 - 85 %
p1 = polyfit(Power_3(1:85),FFR_3(1:85),1);
Power_first = linspace(0,P_max); %kW
FFR_first = polyval(p1,Power_first); %g/s

SFOC_first = FFR_first*3600./Power_first; %g/MWh
Load_first=linspace(0,1); %Load from 0 to 1

% 85 - 100 %
p2 = polyfit(Power_3(86:100),FFR_3(86:100),1);
Power_second = linspace(0,P_max);
FFR_second = polyval(p2,Power_second);

SFOC_second = FFR_second*3600./Power_second;
Load_second=linspace(0,1);

% figure('pos',[0 50 333 600]);
% subplot(2,1,1);
% plot(Power,FFR,'o','MarkerSize',10,'LineWidth',2,'Color','r'); %real point
% grid;
% hold on;
% plot(Power_first(1:85),FFR_first(1:85), 'LineWidth', 2.5, 'Color','b'); %0-85% streight line
% plot(Power_first,FFR_first, '--','LineWidth', 1.5, 'Color','b'); %continuing of the streight line
% plot(Power_second(85:100),FFR_second(85:100), 'LineWidth', 2.5, 'Color','b');%86-100% streight line
% plot(Power_second,FFR_second, '--','LineWidth', 1.5, 'Color','b'); 
% legend('Project Guide data projection', 'Two-piece linear fit', 'Location', 'northwest');
% xlim([0 P_max]);
% ylim([0 250]);
% xlabel('Power [kW]');
% ylabel('Fuel flow rate [g/s]');
% 
% subplot(2,1,2);
% plot(Load_in*100,SFOC_in,'o','MarkerSize',10,'LineWidth',2,'Color','r');
% hold on
% plot(Load_3*100,SFOC_3,'LineWidth', 2.5, 'Color','g');
% hold on
% plot(Load_first(1:85)*100, SFOC_first(1:85), 'LineWidth', 2.5, 'Color','b');
% hold on
% plot(Load_second(85:100)*100, SFOC_second(85:100), 'LineWidth', 2.5, 'Color','b');
% legend('Project Guide data','3rd degree polynomial fit', 'Fuel flow rate linear fit projection');
% xlabel('Load [%]');
% ylabel('Specific fuel consumption [g/kWh]');
% xlim([0 100]);
% ylim([175 270]);
% grid on
% 
% 
% figure('pos',[1200 50 333 600]);
% plot(Load_in*100,SFOC_in,'o','MarkerSize',10,'LineWidth',2,'Color','r');
% hold on
% plot(Power_first(1:85)/P_max*100,  SFOC_first(1:85), '--','LineWidth', 1.5, 'Color','b')
% plot(Power_second(85:100)/P_max*100, SFOC_second(85:100), '--','LineWidth', 1.5, 'Color','b');
% legend('Specific fuel consuption');
% xlabel('Load [%]');
% ylabel('Specific fuel consumption [g/kWh]');
% grid on

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

end

