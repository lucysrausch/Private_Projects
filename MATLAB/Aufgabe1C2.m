samples = 2001;
t = linspace(0, 2, samples);
U = linspace(0, samples, samples);

for i = 1 : samples
   U(i) = sin((i / samples) * 4 * pi);
end

plot(t, U, t, UInt(t, UDot(t, U)), '--', 'LineWidth', 2);
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
title('Aufgabe 2');
legend('U(t)', 'UInt(t, UDot(t, U))');