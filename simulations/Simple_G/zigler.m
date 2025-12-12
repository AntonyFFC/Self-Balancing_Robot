% =========================================================================
% PLIK: dobor_PID_Alokacja.m
% Opis: Analityczny dobór nastaw PID metod¹ Alokacji Biegunów
% =========================================================================
clear all; close all; clc;

% 1. Wczytanie parametrów
if ~exist('a4', 'var')
    run('parametry_robota.m');
end

% Przypomnienie transmitancji uproszczonej G(s) = N(s)/D(s)
% N(s) = b2
% D(s) = a4*s^2 + a2
% Transmitancja PID: C(s) = (Kd*s^2 + Kp*s + Ki) / s

% Równanie charakterystyczne uk³adu zamkniêtego: 
% 1 + G(s)*C(s) = 0  =>  D(s)*s + N(s)*(Kd*s^2 + Kp*s + Ki) = 0

% Po podstawieniu i uporz¹dkowaniu potêg 's':
% (a4 + b2*Kd)*s^3 + (b2*Kp)*s^2 + (a2 + b2*Ki)*s = 0 
% UWAGA: Wyraz wolny musi istnieæ przy PID. 
% Wróæmy do pe³nej formy mianownika:
% Mianownik = s*(a4*s^2 + a2) + b2*(Kd*s^2 + Kp*s + Ki)
% Mianownik = a4*s^3 + a2*s + b2*Kd*s^2 + b2*Kp*s + b2*Ki
% Mianownik = a4 * s^3 + (b2*Kd) * s^2 + (a2 + b2*Kp) * s + (b2*Ki)

% Aby porównywaæ wielomiany, musimy znormalizowaæ (podzieliæ przez a4),
% ¿eby przy s^3 sta³a jedynka.
% s^3 + (b2*Kd/a4)*s^2 + ((a2 + b2*Kp)/a4)*s + (b2*Ki/a4) = 0

% 2. Definicja WYMARZONYCH biegunów (Tu sterujesz zachowaniem robota!)
% Wybieramy 3 bieguny stabilne (ujemne).
% p - okreœla szybkoœæ reakcji. Im bardziej ujemne, tym szybciej (ale potrzeba silniejszych silników).
p = -10; 

% Wielomian po¿¹dany: (s - p)^3 = (s + 10)^3
% (s+10)(s^2 + 20s + 100) = s^3 + 20s^2 + 100s + 10s^2 + 200s + 1000
% = s^3 + 30s^2 + 300s + 1000
% Ogólnie dla (s-p)^3 wzory Viete'a dla wielomianu s^3 + a2*s^2 + a1*s + a0:
target_poly = poly([p p p]); 
% target_poly = [1,  -3*p,  3*p^2,  -p^3]

wanted_a2 = target_poly(2); % Wspó³czynnik przy s^2
wanted_a1 = target_poly(3); % Wspó³czynnik przy s^1
wanted_a0 = target_poly(4); % Wspó³czynnik przy s^0 (wyraz wolny)

fprintf('Chcemy uzyskaæ wielomian: s^3 + %.2fs^2 + %.2fs + %.2f\n', ...
        wanted_a2, wanted_a1, wanted_a0);

% 3. Obliczenie nastaw z porównania wspó³czynników
% Równanie rzeczywiste (znormalizowane przez a4):
% s^3 + (b2*Kd/a4)*s^2 + ((a2 + b2*Kp)/a4)*s + (b2*Ki/a4)

% Porównanie przy s^2: b2*Kd/a4 = wanted_a2
Kd = (wanted_a2 * a4) / b2;

% Porównanie przy s^1: (a2 + b2*Kp)/a4 = wanted_a1
% a2 + b2*Kp = wanted_a1 * a4
Kp = (wanted_a1 * a4 - a2) / b2;

% Porównanie przy s^0: b2*Ki/a4 = wanted_a0
Ki = (wanted_a0 * a4) / b2;

fprintf('\n=== WYLICZONE NASTAWY (Pole Placement) ===\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

% 4. Weryfikacja symulacj¹
simple = false;
if simple
    Gs = tf(b2, [a4, 0, a2]);
else
    b_x = 0.1; b_theta = 0.1; % Tarcie (przyk³adowe)
    Ga4 = Det;
    Ga3 = b_x * J_tot + b_theta * M_tot;
    Ga2 = b_x * b_theta - M_tot * m * g * l;
    Ga1 = -b_x * m * g * l;
    
    % Licznik (b)
    Gb2 = -(M_tot + m*l/R);
    Gb1 = -b_x;
    
    %% 2. Utworzenie Transmitancji Ci¹g³ej G(s)
    num = [Gb2, Gb1];          % Wspó³czynniki przy s^2, s^1, s^0
    den = [Ga4, Ga3, Ga2, Ga1];  % Wspó³czynniki przy s^4, s^3, s^2, s^1, s^0
    
    Gs = tf(num, den);
end
PID_con = pid(Kp, Ki, Kd);
T_closed = feedback(PID_con * Gs, 1);
[y, t] = step(T_closed, 2);

figure;
plot(t, y, 'b', 'LineWidth', 1.5);
title(['OdpowiedŸ skokowa dla biegunów w p = ' num2str(p)]);
xlabel('Czas [s]');
ylabel('K¹t \theta [rad]');
grid on;
print('Odp_skok_ciagly_PID.png', '-dpng', '-r400');

% 5. Mapa biegunów - sprawdzenie czy s¹ tam gdzie chcieliœmy
figure;
pzmap(T_closed);
title('Mapa biegunów uk³adu zamkniêtego');
grid on;
