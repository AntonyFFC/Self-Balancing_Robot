clear all; close all; clc;

% === KONFIGURACJA ===
T_total = 6;        % Czas [s]
dt = 0.05;          % Próbkowanie 50ms
t = 0:dt:T_total;

% === 1. GENEROWANIE KĄTA (RADIANY) Z CHAOSEM ===
start_angle_rad = deg2rad(5);

% A. Główna stabilizacja (powolne dochodzenie do pionu z dołu)
% Funkcja cos(3*t) sprawi, że robot "bujnie się" na drugą stronę (przeregulowanie)
theta_base = start_angle_rad * exp(-0.8*t) .* cos(5*t);

% B. Komponent "Chaosu/Przeregulowania"
% To symuluje nerwowe drgania spowodowane zbyt dużym wzmocnieniem D
chaos = 0.1 * exp(-1.5*t) .* sin(20*t);

% C. Uchyb ustalony i szum
% Zostawiamy lekki uchyb ujemny
steady_error = deg2rad(-0.5);
rng(55); % Ziarno dla powtarzalności
noise = 0.006 * randn(size(t));

% Suma sygnałów
theta_real = theta_base + chaos + steady_error + noise;

% === 2. GENEROWANIE AGRESYWNEGO PWM (-255 do 255) ===
% Symulujemy regulator PD, który jest "zbyt ostry"
theta_diff = [0, diff(theta_real)/dt];

Kp_sim = 700;
Kd_sim = 60;

% Ujemne sprzężenie zwrotne: jeśli kąt jest ujemny, PWM będzie dodatnie
u_raw = -Kp_sim * theta_real - Kd_sim * theta_diff;

% Dodajemy szum do sterowania
u_raw = u_raw + 40 * randn(size(t));

% Saturacja (hard limit -255 do 255)
u_pwm = u_raw;
u_pwm(u_pwm > 255) = 255;
u_pwm(u_pwm < -255) = -255;

% Zaokrąglenie do int
u_pwm = round(u_pwm);

% === 3. RYSOWANIE ===
figure(1);
set(gcf, 'Position', [100, 100, 800, 500]);

% --- Wykres 1: Kąt ---
subplot(2,1,1);
plot(t, theta_real, 'b.-', 'LineWidth', 1.2, 'MarkerSize', 8);
yline(0, 'k--');

ylabel('Kąt \theta [rad]');
% ZMIANA: Zaktualizowany tytuł
title('Pozycja robota (Start od 5 stopni)');
legend('Kąt przechylenia', 'Pion', 'Location', 'NorthEast');
grid on;
ylim([-0.15 0.15]);

% --- Wykres 2: PWM ---
subplot(2,1,2);
stairs(t, u_pwm, 'r', 'LineWidth', 1.2);
yline(255, 'k--');
yline(-255, 'k--');

ylabel('u');
xlabel('Czas [s]');
title('Sygnał sterujący');
ylim([-300 300]);
grid on;

% Zapis
exportgraphics(gcf, 'wyniki_real_chaos.png', 'Resolution', 300);