function [y, t] = PID_c(G_s, Kp, Ki, Kd, czas_symulacji, typ_wymuszenia)

% WEJŚCIA:
%   G_s            - Transmitancja obiektu (klasa tf)
%   Kp, Ki, Kd     - Nastawy regulatora
%   czas_symulacji - (Opcjonalne) Czas końcowy [s] lub wektor czasu
%   typ_wymuszenia - (Opcjonalne) 'step' (skok) lub 'impulse' (impuls)
%
% WYJŚCIA:
%   y - Wektor wyjściowy (odpowiedź układu)
%   t - Wektor czasu

    if nargin < 5 || isempty(czas_symulacji)
        czas_symulacji = 10;
    end
    if nargin < 6
        typ_wymuszenia = 'step';
    end
    
    s = tf('s');
    C_s = Kp + Ki/s + Kd*s;
    
    % Opcjonalnie: Filtr dolnoprzepustowy dla członu D (bardziej realistyczny)
    % N = 100; % Współczynnik filtracji
    % C_s = Kp + Ki/s + (Kd*s)/(1 + s*Kd/N);

    % 3. Obliczenie układu zamkniętego (Sprzężenie zwrotne ujemne)
    % T(s) = (C*G) / (1 + C*G)
    Sys_closed = feedback(C_s * G_s, 1);

    switch lower(typ_wymuszenia)
        case 'step'
            [y, t] = step(Sys_closed, czas_symulacji);
        case 'impulse'
            [y, t] = impulse(Sys_closed, czas_symulacji);
        otherwise
            error('Nieznany typ wymuszenia. Użyj ''step'' lub ''impulse''.');
    end

end