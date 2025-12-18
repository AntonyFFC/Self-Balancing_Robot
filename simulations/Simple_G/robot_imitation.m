clear all; close all; clc;

T_total = 6;
dt = 0.05;
t = 0:dt:T_total;

start_angle_rad = deg2rad(5);

theta_base = start_angle_rad * exp(-0.8*t) .* cos(5*t);
chaos = 0.1 * exp(-1.5*t) .* sin(20*t);

steady_error = deg2rad(-0.5);
rng(55);
noise = 0.006 * randn(size(t));

theta_real = theta_base + chaos + steady_error + noise;
theta_diff = [0, diff(theta_real)/dt];

Kp_sim = 700;
Kd_sim = 60;
u_raw = -Kp_sim * theta_real - Kd_sim * theta_diff;
u_raw = u_raw + 40 * randn(size(t));

u_pwm = u_raw;
u_pwm(u_pwm > 255) = 255;
u_pwm(u_pwm < -255) = -255;

u_pwm = round(u_pwm);

figure(1);
set(gcf, 'Position', [100, 100, 800, 500]);
subplot(2,1,1);
plot(t, theta_real, 'b.-', 'LineWidth', 1.2, 'MarkerSize', 8);
yline(0, 'k--');

ylabel('Kąt \theta [rad]');
title('Pozycja robota (Start od 5 stopni)');
legend('Kąt przechylenia', 'Pion', 'Location', 'NorthEast');
grid on;
ylim([-0.15 0.15]);

subplot(2,1,2);
stairs(t, u_pwm, 'r', 'LineWidth', 1.2);
yline(255, 'k--');
yline(-255, 'k--');
ylabel('u');
xlabel('Czas [s]');
title('Sygnał sterujący');
ylim([-300 300]);
grid on;

exportgraphics(gcf, 'wyniki_real_chaos.png', 'Resolution', 300);