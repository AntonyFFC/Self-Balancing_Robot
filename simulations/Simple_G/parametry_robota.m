% =========================================================================
% PLIK: parametry_robota.m
% Opis: Definicja stałych fizycznych i obliczanie współczynników modelu
% =========================================================================
clear all; clc;

% === 1. PARAMETRY FIZYCZNE ROBOTA ===
g = 9.81;       % Przyspieszenie ziemskie [m/s^2]
m = 1.0;        % Masa wahadła (korpusu) [kg]
l = 0.15;       % Odległość do środka masy [m]
M = 0.5;        % Masa podstawy (kół) [kg]
R = 0.04;       % Promień koła [m]
Jw = 0.0004;    % Moment bezwładności koła [kg*m^2]
Jc = 0.01;      % Moment bezwładności korpusu [kg*m^2]

% Parametr symulacji cyfrowej
Ts = 0.01;     % Czas próbkowania [s] (np. 200 Hz - typowe dla robotów)

% === 2. PARAMETRY POMOCNICZE (Z MODELU) ===
% Masa zastępcza całkowita (translacja)
M_tot = M + m + Jw/(R^2);

% Moment bezwładności całkowity (rotacja wokół osi kół)
J_tot = Jc + m*(l^2);

% Wyznacznik macierzy masowej (a4 w Twoim wyprowadzeniu)
Det = M_tot * J_tot - (m*l)^2;

% === 3. WSPÓŁCZYNNIKI TRANSMITANCJI UPROSZCZONEJ (bez tarcia) ===
% G(s) = b2 / (a4*s^2 + a2)

a4 = Det;                  % Przy s^2 w mianowniku
a2 = -M_tot * m * g * l;   % Wyraz wolny mianownika (ujemny -> niestabilność)
b2 = -(M_tot + (m*l)/R);   % Licznik (stała)

% Wyświetlenie obliczonych wartości
fprintf('=== OBLICZONE PARAMETRY ===\n');
fprintf('M_tot = %.4f\n', M_tot);
fprintf('J_tot = %.4f\n', J_tot);
fprintf('Det (a4) = %.4f\n', a4);
fprintf('a2 = %.4f\n', a2);
fprintf('b2 = %.4f\n', b2);
fprintf('===========================\n');