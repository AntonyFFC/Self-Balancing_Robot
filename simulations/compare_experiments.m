clear; clc; close all;

%% --- CONFIGURATION ---
% Folder containing the '_sliced.mat' files
target_folder = 'csv/1_Ti/';

plot_config = {
    'ti1_rest_optimal_sliced.mat',  '1/Ti = 1',      [0 0.447 0.741]; % Blue
    'ti3td04kp12_sliced.mat',  '1/Ti = 3',    [0.85 0.325 0.098]; % Orange
    'ti6_else_optimal_sliced.mat',  '1/Ti = 6',  [0 0.5 0];       % Green
    'ti_11_else_optimal_sliced',  '1/Ti = 11',      [0 0 0];
};

% files_to_plot = {
%     'Td=0_sliced.mat';  % Will appear 3rd in legend (Yellow)
%     'Td=0.1_sliced.mat';   % Will appear 1st in legend (Blue)
%     'Td=0.4_sliced.mat';   % Will appear 2nd in legend (Orange)
% };

% files_to_plot = {
%     'Kp=6_sliced.mat';   % Will appear 1st in legend (Blue)
%     'Kp=8_sliced.mat';   % Will appear 2nd in legend (Orange)
%     'Kp=12_sliced.mat';  % Will appear 3rd in legend (Yellow)
%     'Kp=16_sliced.mat';  % Will appear 4th in legend (Purple)
% };

% Axis settings
x_tick_step = 2;   % X-axis label every 2 seconds
y_pitch_step = 5;  % Pitch Y-axis label every 5 degrees
y_ctrl_step = 50;  % Control Y-axis label every 50 units

%% --- LOAD DATA ---
file_pattern = fullfile(target_folder, '*.mat');
files = dir(file_pattern);
if isempty(files)
    error('No sliced .mat files found in "%s". Run the previous script first!', data_folder);
end

% Define colors for plotting (Supports up to 7 distinct lines, cycles if more)
colors = {
    [0.0 0.447 0.741];
    [0.85 0.325 0.098];
    [0.0 0.5 0.0];
    };

%% --- PLOTTING ---
figure('Name', 'PID Comparison', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% Initialize Axes
ax1 = subplot(2, 1, 1);
hold(ax1, 'on');
title(ax1, 'Porównanie przebiegów przechylenia');
ylabel(ax1, 'Kąt [\circ]');
grid(ax1, 'on'); grid(ax1, 'minor');
ylim(ax1, [172, 188]);

ax2 = subplot(2, 1, 2);
hold(ax2, 'on');
title(ax2, 'Porównanie sygnału sterującego [u]');
xlabel(ax2, 'Czas [s]');
ylabel(ax2, 'Synał Sterujący');
grid(ax2, 'on'); grid(ax2, 'minor');
ylim(ax2, [-200, 200]);

% Loop through every file and plot
max_time = 0; % Track longest duration for x-axis limit

for i = 1:length(files)
    filename = plot_config{i, 1};
    legend_label = plot_config{i, 2}; % Custom name with slashes allowed
    line_color   = plot_config{i, 3};

    full_path = fullfile(target_folder, filename);
    
    % Load the data
    loaded_data = load(full_path);
    
%     % Generate Display Name (remove '_sliced.mat' for cleaner legend)
%     legend_label = strrep(filename, '.mat', '');        % Remove extension
%     legend_label = strrep(legend_label, '_sliced', ''); % Remove '_sliced'
%     legend_label = strrep(legend_label, '_', ' ');
    
    % Check if variables exist
    if isfield(loaded_data, 't_sliced') && isfield(loaded_data, 'pitch_sliced')
        
        % Plot Pitch
        plot(ax1, loaded_data.t_sliced, loaded_data.pitch_sliced, ...
            'Color', line_color, 'LineWidth', 1.2, 'DisplayName', legend_label);
        
        % Plot Control Signal
        plot(ax2, loaded_data.t_sliced, loaded_data.control_sliced, ...
            'Color', line_color, 'LineWidth', 1.2, 'DisplayName', legend_label);
        
        % Update max time found
        if max(loaded_data.t_sliced) > max_time
            max_time = max(loaded_data.t_sliced);
        end
    else
        warning('File %s does not contain the required variables.', files(i).name);
    end
end

% Add Target Pitch Line (Set Point) to the top plot only once (assuming it's same for all)
% We use the last loaded data to draw the reference line
if exist('loaded_data', 'var')
    yline(ax1, mean(loaded_data.set_pitch_sliced), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Zadany kąt');
end

% Zero line for Control Signal
yline(ax2, 0, 'k-', 'Alpha', 0.3, 'HandleVisibility', 'off');

%% --- FINAL FORMATTING ---

% Apply Legends
legend(ax1, 'Interpreter', 'none', 'Location', 'best');
legend(ax2, 'Interpreter', 'none', 'Location', 'best');

% Apply Ticks and Limits
xticks(ax1, 0 : x_tick_step : ceil(max_time));
yticks(ax1, 170 : y_pitch_step : 190);
xlim(ax1, [0, max_time]);

xticks(ax2, 0 : x_tick_step : ceil(max_time));
yticks(ax2, -200 : y_ctrl_step : 200);
xlim(ax2, [0, max_time]);

hold(ax1, 'off');
hold(ax2, 'off');