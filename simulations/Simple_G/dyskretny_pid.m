% =========================================================================
% PLIK: symulacja_cyfrowa_PID.m
% Opis: Pełna symulacja pętli sterowania z dyskretnym regulatorem PID
%       i dyskretnym modelem obiektu (odwróconego wahadła).
% =========================================================================
clear all; close all; clc;

% === 1. WCZYTANIE PARAMETRÓW FIZYCZNYCH ===
% Upewnij się, że plik parametry_robota.m jest w tym samym folderze
run('parametry_robota.m'); 

% === 2. SYNTEZA REGULATORA (ALOKACJA BIEGUNÓW) ===
% Powtórzenie obliczeń, aby mieć świeże Kp, Ki, Kd
p = -10; % Pożądany biegun (szybkość reakcji) - MOŻESZ ZMIENIAĆ

% Wielomian pożądany (s-p)^3
target_poly = poly([p p p]);
wanted_a2 = target_poly(2);
wanted_a1 = target_poly(3);
wanted_a0 = target_poly(4);

% Obliczenie nastaw ciągłych (równoległych)
Kd_cont = (wanted_a2 * a4) / b2;
Kp_cont = (wanted_a1 * a4 - a2) / b2;
Ki_cont = (wanted_a0 * a4) / b2;

fprintf('=== NASTAWY CIĄGŁE ===\n');
fprintf('Kp = %.4f, Ki = %.4f, Kd = %.4f\n', Kp_cont, Ki_cont, Kd_cont);

% === 3. OBLICZENIE WSPÓŁCZYNNIKÓW DYSKRETNYCH (r0, r1, r2) ===
% Konwersja na czasy (wartości bezwzględne)
Kp_abs = abs(Kp_cont);
Ti = Kp_abs / abs(Ki_cont);
Td = abs(Kd_cont) / Kp_abs;

% Wzory hybrydowe (Trapez dla I, Euler dla D)
r0 = Kp_abs * (1 + Ts/(2*Ti) + Td/Ts);
r1 = Kp_abs * (Ts/(2*Ti) - (2*Td)/Ts - 1);
r2 = (Kp_abs * Td) / Ts;

fprintf('\n=== WSPÓŁCZYNNIKI CYFROWE ===\n');
fprintf('r0 = %.4f\n', r0);
fprintf('r1 = %.4f\n', r1);
fprintf('r2 = %.4f\n', r2);

% === 4. PRZYGOTOWANIE MODELU DYSKRETNEGO OBIEKTU ===
% Zgodnie z wyprowadzeniem w pracy (uproszczony model 2. rzędu)
% y(k) = (1/W) * [ P1*y(k-1) + P2*y(k-2) + Q0*u(k) ]

W  = a4 + a2 * Ts^2;
P1 = 2 * a4;
P2 = -a4;
Q0 = b2 * Ts^2;

% === 5. INICJALIZACJA SYMULACJI ===
T_sim = 2.0;            % Czas symulacji [s]
N = round(T_sim / Ts);  % Liczba próbek
time = 0:Ts:T_sim;      % Wektor czasu

% Tablice na wyniki (zainicjowane zerami)
theta = zeros(1, N+1);  % Wyjście obiektu (kąt)
u     = zeros(1, N+1);  % Sterowanie (napięcie/PWM)
e     = zeros(1, N+1);  % Uchyb regulacji

% Warunki początkowe
% Symulujemy popchnięcie robota: zaczyna od wychylenia 5 stopni (0.087 rad)
theta(1) = 0.087; 
theta(2) = 0.087; % Zakładamy zerową prędkość na starcie (brak zmiany)

target_angle = 0.0; % Cel: pion (0 radianów)

% === 6. PĘTLA GŁÓWNA (Symulacja działania mikrokontrolera) ===
fprintf('\nRozpoczynam symulację cyfrową...\n');

for k = 3:N+1
    % --- KROK 1: POMIAR I UCHYB ---
    % W rzeczywistości tu czytałbyś z IMU/enkodera.
    % Tutaj bierzemy wynik z poprzedniego kroku symulacji.
    measurement = theta(k-1); 
    
    e(k) = target_angle - measurement;
    
    % --- KROK 2: ALGORYTM REGULATORA (Równanie różnicowe) ---
    % u(k) = u(k-1) + r0*e(k) + r1*e(k-1) + r2*e(k-2)
    
    % Ponieważ obiekt ma ujemne wzmocnienie, a my policzyliśmy r0,r1,r2 jako dodatnie,
    % musimy zmienić znak sterowania (ujemne sprzężenie zwrotne),
    % LUB użyliśmy ujemnych Kp,Ki,Kd. 
    % Zastosujmy podejście standardowe: ujemne Kp w alokacji oznacza, 
    % że regulator "kontruje".
    
    % UWAGA IMPLEMENTACYJNA: 
    % W alokacji wyszły nam ujemne Kp, Ki, Kd.
    % We wzorach na r0, r1, r2 użyliśmy Kp_abs (dodatnie).
    % Więc musimy dodać minus przy sumowaniu, aby zachować ujemne sprzężenie.
    
    delta_u = r0*e(k) + r1*e(k-1) + r2*e(k-2);
    u(k) = u(k-1) - delta_u; % Minus, bo r0,r1,r2 są dodatnie, a potrzebujemy ujemnej reakcji
    
    % (Opcjonalnie) Ograniczenie sterowania (Saturacja silników)
    % np. +/- 12 Volt
    if u(k) > 12
        u(k) = 12;
    elseif u(k) < -12
        u(k) = -12;
    end
    
    % --- KROK 3: FIZYKA OBIEKTU (MODEL DYSKRETNY) ---
    % y(k) = (1/W) * [ P1*y(k-1) + P2*y(k-2) + Q0*u(k) ]
    
    theta(k) = (1/W) * ( P1*theta(k-1) + P2*theta(k-2) + Q0*u(k) );
end

% === 7. WYKRESY ===
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