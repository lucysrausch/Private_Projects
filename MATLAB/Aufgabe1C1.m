samples = 2001;
t = linspace(0, 2, samples);           %vector for time
U = linspace(0, samples, samples);     %empty vector for voltage
%fill the vector with a sine function
for i = 1 : samples
   U(i) = sin((i / samples) * 4 * pi); 
end

%plot the original function as well as the derivation of it.
plot(t, U, t, UDot(t, U), 'LineWidth', 2);
set(gca, 'FontSize', 15);  %change fontsize of the axis labeling
xlabel('Zeit in s');       %set x-axis label
title('Aufgabe 1');        %set title
legend('U(t)', 'UDot(t)'); %add legend
