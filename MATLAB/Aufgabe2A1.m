samples=2001;

t = linspace(0, 2, samples);

U = zeros(1, samples);

index = find(t >= 0.5, 1);

U(index:samples) = 1;

C = 500e-6;
L = 300e-3;

subplot(1,3,1);
plot(t, U, 'LineWidth', 2);
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Spannung');
title('Spannung', 'FontSize', 20);

subplot(1,2,2);
plot(t, C * UDot(t, U), 'LineWidth', 2);
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Strom');
title('Kondensator', 'FontSize', 20);

figure;
subplot(1,2,1);
plot(t, U, 'LineWidth', 2);
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Spannung');
title('Spannung', 'FontSize', 20);

subplot(1,2,2);
plot(t, UInt(t, U) / L, 'LineWidth', 2);
set(gca, 'FontSize', 15);
xlabel('Zeit in s');
ylabel('Strom');
title('Spule', 'FontSize', 20);



