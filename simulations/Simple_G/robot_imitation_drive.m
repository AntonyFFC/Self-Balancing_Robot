clear all; close all; clc;

% === KONFIGURACJA ===
T_total = 10;       % Dłuższy czas, żeby pokazać jazdę
dt = 0.05;          % Próbkowanie 50ms
t = 0:dt:T_total;

% === 1. GENEROWANIE WARTOŚCI ZADANEJ (PROFIL TRAPEZOWY) ===
% Definiujemy kluczowe punkty w czasie
t_points = [0,  2,   3,   6,   7,   10]; 
% Definiujemy wartości zadane kąta w tych punktach (rad)
% 0 -> Start rampy -> Max wychylenie -> Koniec jazdy -> Stop rampy -> 0
angle_drive = 0.1; % Kąt jazdy (ok. 8.5 stopnia)
theta_points = [0,  0,   angle_drive, angle_drive, 0,    0];

% Interpolacja liniowa (tworzy idealny profil zadany)
theta_ref = interp1(t_points, theta_points, t, 'linear');

% === 2. GENEROWANIE RZECZYWISTEGO KĄTA (Z WŁADNOŚCIĄ I CHAOSEM) ===
% Robot nie nadąża idealnie za zadaniem (bezwładność)
% Symulujemy to filtrem dolnoprzepustowym + oscylacje

theta_real = zeros(size(t));
velocity = 0;

for k = 2:length(t)
    % Prosta fizyka: dążenie do theta_ref z opóźnieniem
    accel = 5 * (theta_ref(k) - theta_real(k-1)) - 2 * velocity;
    velocity = velocity + accel * dt;
    theta_real(k) = theta_real(k-1) + velocity * dt;
end

% Dodajemy "brudne" efekty rzeczywiste
rng(123); 
% Szum pomiarowy
noise = 0.005 * randn(size(t));
% Drgania mechaniczne (silniejsze podczas ruchu)
vibration = 0.003 * sin(30*t) .* (abs(theta_real) > 0.01); 
% Uchyb ustalony (robot zawsze trochę krzywy)
steady_error = -0.01; 

theta_real = theta_real + noise + vibration + steady_error;


% === 3. GENEROWANIE SYGNAŁU STERUJĄCEGO (PWM) ===
% Uchyb regulacji
error = theta_ref - theta_real;
% Pochodna uchybu (zmiana błędu w czasie)
error_diff = [0, diff(error)/dt];

Kp = 2000;
Kd = 150; 

% Obliczenie sterowania PD
u_raw = Kp * error + Kd * error_diff;

% Dodanie szumu do sterowania
u_raw = u_raw + 50 * randn(size(t));

% Saturacja PWM (-255 do 255)
u_pwm = u_raw;
u_pwm(u_pwm > 255) = 255;
u_pwm(u_pwm < -255) = -255;
u_pwm = round(u_pwm);


% === 4. RYSOWANIE WYKRESÓW ===
figure(1);
set(gcf, 'Position', [100, 100, 900, 600]);

% --- Wykres 1: Kąt (Zadany vs Rzeczywisty) ---
% subplot(2,1,1);
% Rysujemy wartość zadaną (przerywana linia)
plot(t, theta_ref, 'k--', 'LineWidth', 2); 
hold on;
% Rysujemy rzeczywisty pomiar
plot(t, theta_real, 'b.-', 'LineWidth', 1, 'MarkerSize', 8);

ylabel('Kąt \theta [rad]');
title('Pozycja robota podczas jazdy');
legend('Wartość Zadana', 'Rzeczywisty Kąt Robota', 'Location', 'NorthWest');
grid on;
ylim([-0.05 0.25]); % Dopasowanie zakresu

% % --- Wykres 2: PWM ---
% subplot(2,1,2);
% stairs(t, u_pwm, 'r', 'LineWidth', 1.2);
% yline(255, 'k--');
% yline(-255, 'k--');
% ylabel('PWM');
% xlabel('Czas [s]');
% title('Sygnał sterujący');
% ylim([-300 300]);
% grid on;

% Zapis
exportgraphics(gcf, 'wyniki_jazda_przod.png', 'Resolution', 300);