%%
data = csvread('tilt.csv');
electric_angle = data(:, 1);
pot = data(:, 2);


plot(electric_angle, pot);
p = polyfit(pot, electric_angle, 1);
p = [round(p(1), 3), round(p(2))];

pot2 = 1:4095;
ea2 = polyval(p, pot2);

plot(electric_angle, pot, ea2, pot2);
