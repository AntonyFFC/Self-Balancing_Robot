clear all; close all; clc;

if ~exist('a4', 'var')
    run('parametry_robota.m');
end
p = -10; 

target_poly = poly([p p p]); 
% target_poly = [1,  -3*p,  3*p^2,  -p^3]

wanted_a2 = target_poly(2);
wanted_a1 = target_poly(3);
wanted_a0 = target_poly(4);

fprintf('Chcemy uzyskaæ wielomian: s^3 + %.2fs^2 + %.2fs + %.2f\n', ...
        wanted_a2, wanted_a1, wanted_a0);

Kd = (wanted_a2 * a4) / b2;
Kp = (wanted_a1 * a4 - a2) / b2;
Ki = (wanted_a0 * a4) / b2;

fprintf('\n=== WYLICZONE NASTAWY (Pole Placement) ===\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

simple = false;
if simple
    Gs = tf(b2, [a4, 0, a2]);
else
    b_x = 0.1; b_theta = 0.1;
    Ga4 = Det;
    Ga3 = b_x * J_tot + b_theta * M_tot;
    Ga2 = b_x * b_theta - M_tot * m * g * l;
    Ga1 = -b_x * m * g * l;
    
    Gb2 = -(M_tot + m*l/R);
    Gb1 = -b_x;
    
    num = [Gb2, Gb1];
    den = [Ga4, Ga3, Ga2, Ga1];
    
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

figure;
pzmap(T_closed);
title('Mapa biegunów uk³adu zamkniêtego');
grid on;
