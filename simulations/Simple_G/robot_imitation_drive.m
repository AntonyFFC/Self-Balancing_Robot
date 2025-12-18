clear all; close all; clc;

T_total = 10;
dt = 0.05;
t = 0:dt:T_total;

t_points = [0,  2,   3,   6,   7,   10]; 
angle_drive = 0.1;
theta_points = [0,  0,   angle_drive, angle_drive, 0,    0];

theta_ref = interp1(t_points, theta_points, t, 'linear');

theta_real = zeros(size(t));
velocity = 0;

for k = 2:length(t)
    accel = 5 * (theta_ref(k) - theta_real(k-1)) - 2 * velocity;
    velocity = velocity + accel * dt;
    theta_real(k) = theta_real(k-1) + velocity * dt;
end

rng(123); 
noise = 0.005 * randn(size(t));
vibration = 0.003 * sin(30*t) .* (abs(theta_real) > 0.01); 
steady_error = -0.01; 

theta_real = theta_real + noise + vibration + steady_error;

error = theta_ref - theta_real;
error_diff = [0, diff(error)/dt];

Kp = 2000;
Kd = 150; 

u_raw = Kp * error + Kd * error_diff;

u_raw = u_raw + 50 * randn(size(t));

u_pwm = u_raw;
u_pwm(u_pwm > 255) = 255;
u_pwm(u_pwm < -255) = -255;
u_pwm = round(u_pwm);


figure(1);
set(gcf, 'Position', [100, 100, 900, 600]);

plot(t, theta_ref, 'k--', 'LineWidth', 2); 
hold on;
plot(t, theta_real, 'b.-', 'LineWidth', 1, 'MarkerSize', 8);

ylabel('Kąt \theta [rad]');
title('Pozycja robota podczas jazdy');
legend('Wartość Zadana', 'Rzeczywisty Kąt Robota', 'Location', 'NorthWest');
grid on;
ylim([-0.05 0.25]);

% subplot(2,1,2);
% stairs(t, u_pwm, 'r', 'LineWidth', 1.2);
% yline(255, 'k--');
% yline(-255, 'k--');
% ylabel('PWM');
% xlabel('Czas [s]');
% title('Sygnał sterujący');
% ylim([-300 300]);
% grid on;

exportgraphics(gcf, 'wyniki_jazda_przod.png', 'Resolution', 300);