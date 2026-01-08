clear; clc; close all;

filename = 'csv/pid_data_share.csv';
dt = 0.100;
try
    data = readmatrix(filename);
catch
    error('File not found! Make sure "%s" is in the current MATLAB folder.', filename);
end

time = data(:, 1);
pitch = data(:, 2);
set_pitch = data(:, 3);

num_samples = length(pitch);
t = 0 : dt : (num_samples - 1) * dt;

% if max(t) > 1000
%     t = (t - t(1)) / 1000.0;
% end

figure('Name', 'Robot Balancing Performance', 'Color', 'w');

plot(t, pitch, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Pitch'); 
hold on;

plot(t, set_pitch, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Set Pitch (Target)');

title('Self-Balancing Robot: Pitch Response');
xlabel('Time (seconds)');
ylabel('Angle (degrees)');
legend('Location', 'best');
grid on;
grid minor;
xlim([0 max(t)]);

yline(0, 'k-', 'Alpha', 0.3, 'HandleVisibility', 'off'); 

hold off;