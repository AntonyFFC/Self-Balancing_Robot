function [y, t] = PID_c(G_s, Kp, Ki, Kd, czas_symulacji, typ_wymuszenia)


    if nargin < 5 || isempty(czas_symulacji)
        czas_symulacji = 10;
    end
    if nargin < 6
        typ_wymuszenia = 'step';
    end
    
    s = tf('s');
    C_s = Kp + Ki/s + Kd*s;
    
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