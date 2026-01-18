clear; clc; close all;

filename = 'csv/pid_data_share.csv';
dt = 0.100;

window_start_offset = -20.5; 
window_end_offset = -0.5;   

x_tick_step = 0.5;
y_pitch_step = 5;
y_ctrl_step = 50;


try
    data = readmatrix(filename);
catch
    error('File not found', filename);
end

raw_pitch = data(:, 2);
raw_set_pitch = data(:, 3);
raw_control = data(:, 4);

num_samples = length(raw_pitch);
full_t = 0 : dt : (num_samples - 1) * dt;

total_duration = full_t(end);
t_start = max(0, total_duration + window_start_offset);
t_end   = total_duration + window_end_offset;

mask = (full_t >= t_start) & (full_t <= t_end);

t_sliced = full_t(mask);
t_sliced = t_sliced - t_sliced(1);
pitch_sliced = raw_pitch(mask);
set_pitch_sliced = raw_set_pitch(mask);
control_sliced = raw_control(mask);

figure('Name', 'Robot Analysis', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

subplot(2, 1, 1);
plot(t_sliced, pitch_sliced, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Pitch'); 
hold on;
plot(t_sliced, set_pitch_sliced, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Set Pitch');
hold off;

title(['Pitch Response (' filename ')']);
ylabel('Angle (degrees)');
legend('Location', 'best');
grid on; grid minor;

xlim([t_sliced(1), t_sliced(end)]);
ylim([150, 200]);
xticks(t_sliced(1) : x_tick_step : t_sliced(end));
yticks(150 : y_pitch_step : 200);

subplot(2, 1, 2);
plot(t_sliced, control_sliced, 'g-', 'LineWidth', 1.0, 'DisplayName', 'Control Signal');
hold on;
yline(0, 'k-', 'Alpha', 0.3, 'HandleVisibility', 'off');
hold off;

title('Control Signal Output');
xlabel('Time (seconds)');
ylabel('Control Signal');
legend('Location', 'best');
grid on; grid minor;
xlim([t_sliced(1), t_sliced(end)]);
ylim([-255, 255]);
xticks(t_sliced(1) : x_tick_step : t_sliced(end));
yticks(-255 : y_ctrl_step : 255);

[filepath, name, ext] = fileparts(filename);
save_filename = fullfile(filepath, [name '_sliced.mat']);

save(save_filename, 't_sliced', 'pitch_sliced', 'set_pitch_sliced', 'control_sliced', 'filename');

fprintf('Successfully saved cut data to: %s\n', save_filename);