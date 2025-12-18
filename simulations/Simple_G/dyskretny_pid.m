clear all; close all; clc;

run('parametry_robota.m'); 

p = -10; % Pożądany biegun

% Wielomian pożądany (s-p)^3
target_poly = poly([p p p]);
wanted_a2 = target_poly(2);
wanted_a1 = target_poly(3);
wanted_a0 = target_poly(4);

Kd_cont = (wanted_a2 * a4) / b2;
Kp_cont = (wanted_a1 * a4 - a2) / b2;
Ki_cont = (wanted_a0 * a4) / b2;

fprintf('NASTAWY CIĄGŁE \n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp_cont, Ki_cont, Kd_cont);

Kp_abs = abs(Kp_cont);
Ti = Kp_abs / abs(Ki_cont);
Td = abs(Kd_cont) / Kp_abs;

r0 = Kp_abs * (1 + Ts/(2*Ti) + Td/Ts);
r1 = Kp_abs * (Ts/(2*Ti) - (2*Td)/Ts - 1);
r2 = (Kp_abs * Td) / Ts;

fprintf('\n NASTAWY DYSKRETNE \n');
fprintf('r0 = %.4f\n', r0);
fprintf('r1 = %.4f\n', r1);
fprintf('r2 = %.4f\n', r2);

% y(k) = (1/W) * [ P1*y(k-1) + P2*y(k-2) + Q0*u(k) ]

W  = a4 + a2 * Ts^2;
P1 = 2 * a4;
P2 = -a4;
Q0 = b2 * Ts^2;

T_sim = 2.0;
N = round(T_sim / Ts);
time = 0:Ts:T_sim;

theta = zeros(1, N+1);
u = zeros(1, N+1);
e = zeros(1, N+1);

theta(1) = 0.087; % 5 stopni
theta(2) = 0.087; % Zakładam zerową prędkość na starcie

target_angle = 0.0;

for k = 3:N+1
    measurement = theta(k-1); 
    
    e(k) = target_angle - measurement;
    
    % W alokacji wyszło nam ujemne Kp, Ki, Kd.
    % we wzorach na r0, r1, r2 użyłkem absolutne wartości Kp_abs.
    % więc trzeba dodać minus przy sumowaniu, aby zachować ujemne sprzężenie.
    
    delta_u = r0*e(k) + r1*e(k-1) + r2*e(k-2);
    u(k) = u(k-1) - delta_u;
    
    if u(k) > 12
        u(k) = 12;
    elseif u(k) < -12
        u(k) = -12;
    end
    
    % y(k) = (1/W) * [ P1*y(k-1) + P2*y(k-2) + Q0*u(k) ]
    
    theta(k) = (1/W) * ( P1*theta(k-1) + P2*theta(k-2) + Q0*u(k) );
end

figure('Position', [100, 100, 600, 250]);
stairs(time, theta, 'LineWidth', 2);
yline(0, 'k--');
xlabel('Czas [s]');
ylabel('Kąt \theta [rad]');
title(['Pozycja robota (Start od 5 stopni, p = ' num2str(p) ')']);
grid on;
print('Pozycja_dyskretny_PID.png', '-dpng', '-r400');

figure('Position', [100, 100, 600, 250]);
stairs(time, u, 'r', 'LineWidth', 1.5);
xlabel('Czas [s]');
ylabel('Sterowanie u [Nm]');
title('Sygnał sterujący');
print('Sterowanie_dyskretny_PID.png', '-dpng', '-r400');
grid on;