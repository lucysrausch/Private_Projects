samples = 2001;
t = linspace(0, 2, samples);
U = linspace(0, samples, samples);

for i = 1 : samples
   U(i) = sin((i / samples) * 4 * pi);
end

plot(t,U,t,UDot(t, U),t,UDot(t, U)), 'LineWidth', 2);
xlabel('time');
ylabel('voltage');
title('yay');
