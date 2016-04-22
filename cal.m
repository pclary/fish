%%
data = csvread('pan.csv');
electric_angle = data(:, 1);
pot = data(:, 2);


plot(electric_angle, pot);
p = polyfit(pot, electric_angle, 1);

pot2 = 1:4095;
ea2 = (pot2 - 2215)*0.568;

plot(electric_angle, pot, ea2, pot2);
