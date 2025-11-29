% === PARAMETRY FIZYCZNE ROBOTA ===
g = 9.81;       % Przyspieszenie ziemskie [m/s^2]
m = 1.0;        % Masa wahadła (korpusu) [kg]
l = 0.15;       % Odległość do środka masy [m]
M = 0.5;        % Masa podstawy (kół) [kg]
R = 0.04;       % Promień koła [m]
Jw = 0.0004;    % Moment bezwładności koła [kg*m^2]
Jc = 0.01;      % Moment bezwładności korpusu [kg*m^2]
b_x = 0.1; b_theta = 0.1; % Tarcie (przykładowe)
% b_x = 0; b_theta = 0;

% Stała grawitacyjna (maksymalny moment, jaki może utrzymać robot)
% Odpowiednik parametru skalującego charakterystykę
MGL = m * g * l; 

% Punkty linearyzacji (sterowanie u w Nm)
% Uwaga: u musi być mniejsze niż MGL (ok. 1.47 Nm), inaczej robot upada
ulin1 = -1;  % Punkt ujemny (wychylenie w tył)
ulin2 = 0;           % Pion (punkt równowagi)
ulin3 = 0.7;   % Punkt dodatni (wychylenie w przód)

% Zakres sterowania do wykresów (nieco mniej niż granica przewrócenia)
u_range_limit = 0.99 * MGL;


% Stałe pomocnicze
M_tot = M + m + Jw/R^2;
J_tot = Jc + m*l^2;
Det = M_tot * J_tot - (m*l)^2;

% === OBLICZENIE MACIERZY A i B ===
% Macierz A (4x4)
A = zeros(4,4);
A(1,3) = 1;
A(2,4) = 1;
A(3,2) = -(m*l)^2 * g / Det;
A(3,3) = -b_x * J_tot / Det;
A(3,4) = b_theta * m * l / Det;
A(4,2) = M_tot * m * g * l / Det;
A(4,3) = b_x * m * l / Det;
A(4,4) = -b_theta * M_tot / Det;

% Macierz B (4x1)
B = zeros(4,1);
B(3) = (J_tot/R + m*l) / Det;
B(4) = -(M_tot + m*l/R) / Det;

C = [0 1 0 0]; 
D = 0;

% === 3. PRZYPISANIE DO ZMIENNYCH DLA SIMULINKA ===

a32 = A(3,2);
a33 = A(3,3);
a34 = A(3,4);

a42 = A(4,2);
a43 = A(4,3);
a44 = A(4,4);

b3 = B(3);
b4 = B(4);

% Wyświetlenie komunikatu
disp('Parametry załadowane do Workspace. Klocki w Simulinku powinny przestać być czerwone.');