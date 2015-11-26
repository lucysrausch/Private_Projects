samples=2001;

t = linspace(0, 2, samples);               %vector for time
U = zeros(1, samples);                     %empty vector for voltage

%find the first value in the time vector that is >= 0.5
index = find(t >= 0.5, 1);                 

%set all values of U to 1 that relate to a time >= 0.5
U(index:samples) = 1;                      

C = 500e-6;  % = 500uF
L = 300e-3;  % = 500mH

subplot(1,2,1);                            %subplot, two plots
plot(t, U, 'LineWidth', 2);                %plot U(t) with linewidth 2
set(gca, 'FontSize', 15);                  %fontsize of the axis label
xlabel('Zeit in s');                       %set x-axis label
ylabel('Spannung');                        %set y-axis label
title('Spannung', 'FontSize', 20);         %set title with fontsize 20

subplot(1,2,2);
plot(t, C * UDot(t, U), 'LineWidth', 2);   %plot the current of the cap
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Strom');
title('Kondensator', 'FontSize', 20);

figure;                                    %new plot window
subplot(1,2,1);                            %subplot, two plots
plot(t, U, 'LineWidth', 2);                %plot the voltage
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Spannung');
title('Spannung', 'FontSize', 20);

subplot(1,2,2);
plot(t, UInt(t, U) / L, 'LineWidth', 2);   %plot I(t) of the inductor
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Strom');
title('Spule', 'FontSize', 20);



