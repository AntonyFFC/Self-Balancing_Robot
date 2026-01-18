clear all; clc;

g = 9.81;       % Przyspieszenie ziemskie [m/s^2]
m = 0.28;        % Masa wahadła (korpusu) [kg]
l = 0.07;       % Odległość do środka masy [m]
M = 0.015;        % Masa podstawy (kół) [kg]
R = 0.04;       % Promień koła [m]
Jw = 0.0004;    % Moment bezwładności koła [kg*m^2]
Jc = 0.01;      % Moment bezwładności korpusu [kg*m^2]

Ts = 0.005;     % Czas próbkowania [s]

M_tot = M + m + Jw/(R^2);
J_tot = Jc + m*(l^2);
Det = M_tot * J_tot - (m*l)^2;

% TRANSMITANCJA UPROSZCZONA (bez tarcia)
% G(s) = b2 / (a4*s^2 + a2)

a4 = Det;
a2 = -M_tot * m * g * l;
b2 = -(M_tot + (m*l)/R);

fprintf('=== OBLICZONE PARAMETRY ===\n');
fprintf('M_tot = %.4f\n', M_tot);
fprintf('J_tot = %.4f\n', J_tot);
fprintf('Det (a4) = %.4f\n', a4);
fprintf('a2 = %.4f\n', a2);
fprintf('b2 = %.4f\n', b2);
fprintf('===========================\n');