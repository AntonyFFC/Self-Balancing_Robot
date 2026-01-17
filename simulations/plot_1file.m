clear; clc; close all;

%% --- CONFIGURATION ---
% 1. Select the file you want to plot
filename = 'csv/drive_sliced.mat'; 

% 2. Custom Label (Optional) - Leave empty '' to use filename

% 3. Visual Settings
x_tick_step = 2;   
y_pitch_step = 5;  
y_ctrl_step = 50; 

%% --- LOAD & PLOT ---
if ~isfile(filename)
    error('File not found: %s', filename);
end

data = load(filename);

figure('Name', 'Single Experiment Analysis', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

ax1 = subplot(2, 1, 1);
plot(data.t_sliced, data.pitch_sliced, 'LineWidth', 1.5,'DisplayName', 'Kąt przechylenia');
hold on;

if isfield(data, 'set_pitch_sliced')
    plot(data.t_sliced, data.set_pitch_sliced, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Zadany kąt');
end

title(['Przechylenie i zadany kąt']);
ylabel('Kąt [\circ]');
legend('Location', 'best');
grid on; grid minor;
xlim([0, max(data.t_sliced)]);
ylim([170, 190]);
xticks(0 : x_tick_step : ceil(max(data.t_sliced)));
yticks(170 : y_pitch_step : 190);

ax2 = subplot(2, 1, 2);
plot(data.t_sliced, data.control_sliced, 'LineWidth', 1.2);
hold on;
yline(0, 'k-', 'Alpha', 0.3, 'HandleVisibility', 'off');

title('Sygnał sterujący');
xlabel('Czas [s]');
ylabel('Sygnał sterujący [u]');
grid on; grid minor;
xlim([0, max(data.t_sliced)]);
ylim([-200, 200]);
xticks(0 : x_tick_step : ceil(max(data.t_sliced)));
yticks(-200 : y_ctrl_step : 200);