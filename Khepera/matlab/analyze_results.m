% Create an empty table to store experimental data
clear all; clc; close all;
dataTable = table();

%% load the data
% the general folder
work_station = "Desktop";
% work_station = "Laptop"
if work_station == "Desktop"
    root_path = '/home/wjin/Workspace/continuous_STE/Results';
else
    root_path = '/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera';
end
general_folder=[root_path, '/RAL_2025_results/baseline_exp/Results_2025-01-10/calibrate/'];
folder_list = dir([general_folder '/**']);
folder_list = folder_list(~ismember({folder_list.name},{'.','..', '101', '102','103', 'pc', 'k4'}));
folder_list = folder_list(~ismember({folder_list.folder}, general_folder));
folder_list = folder_list(~ismember({folder_list.folder}, [general_folder '/bad']));
folder_list = folder_list([folder_list.isdir]);
numExperiments = length(folder_list);

for i = 1:numExperiments
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    folder_name_k4 = [folder_name, '/k4'];
    folder_name_pc = [folder_name, '/pc'];

    % ground truth and other params
    params = my_text2struct([folder_name_pc, '/param_log.csv']);
    params_k4 = my_text2struct([folder_name_k4, '/param_log.csv']);
    map_nbr = params.map_nbr;
    % stats
    expected_a_posteriori_file = dlmread([folder_name_pc, '/expected_a_posteriori.csv']);
    iteration_nbr = size(expected_a_posteriori_file,1);
    expected_a_posteriori(1:2) = expected_a_posteriori_file(end,2:3);
    % error on X and Y (col 5 and 6)
    max_a_post_file = dlmread([folder_name_pc, '/max_a_posteriori.csv']);
    if(isnan(expected_a_posteriori(1))), expected_a_posteriori(1) = max_a_post(end,2); end
    if(isnan(expected_a_posteriori(2))), expected_a_posteriori(2) = max_a_post(end,3); end
    source_postion_file_name = [folder_name_pc '/source_position.txt'];
    source_file = fopen(source_postion_file_name, 'r');
    source_real_position = fscanf(source_file,'%f %f');
    error(1) = source_real_position(1) - expected_a_posteriori(1);
    error(2) = source_real_position(2) - expected_a_posteriori(2);
    entropy_file = dlmread([folder_name_pc, '/entropy.csv']);
    entropy(1) = entropy_file(end,1);

    % maximal measurement
    measurement_his=dlmread([folder_name_pc, '/measurement_history.csv']);
    max_meas = max(measurement_his(:,2)); 
    
    max_a_post(1:2) = max_a_post_file(end,2:3);
    convergence = sqrt((max_a_post(1) - expected_a_posteriori(1))^2 + (max_a_post(2) - expected_a_posteriori(2))^2);
    % experimental time
    goal_positions = dlmread([folder_name_k4, '/goal_position.csv']);
    exp_time = (goal_positions(end, 5) - goal_positions(1,5))/1000;
    
    % trajectory_length
    trajectory = dlmread([folder_name_k4, '/trajectory.csv']);
    traj_length_sum = 0;
    for iter = 2:size(trajectory,1)
        traj_length_sum = traj_length_sum + sqrt((trajectory(iter,2)-trajectory(iter-1,2))^2 + (trajectory(iter,3) - trajectory(iter-1,3))^2); 
    end
    traj_length = traj_length_sum;

    
    % strategy
    sampling_mode_idx = params_k4.sampling_mode;
    %sampling_mode_idx = 1;
    sampling_mode = "";
    switch sampling_mode_idx
        case 0
            sampling_mode = "stop sense go";
        case 1
            sampling_mode = "visit sense go";
        case 2
            sampling_mode = "sense in motion";
    end

    planning_mode_idx = params.planning_strategy;
    planning_mode = "";
    switch planning_mode_idx
        case 0
            planning_mode = "greedy";
        case 5
            planning_mode = "belief clustering";
        case 1
            planning_mode = "informative kld";
    end
    % sensor calibration
    calibration_mode_idx = params_k4.sensor_calibration;
    calibration_mode = "";
    switch calibration_mode_idx
        case 0
            calibration_mode = "calibration on";
        case 1 
            calibration_mode = "calibration off";
    end
    % gas feature
    gas_feature_idx = params.gas_feature;
    gas_feature = "";
    switch gas_feature_idx
        case 0
            gas_feature = "gas concentration";
        case 2
            gas_feature = "gas reading rank";
    end
    % sensor 
    % Create a temporary table for the current experiment
    tempTable = table(map_nbr, error(1), error(2), iteration_nbr, exp_time, traj_length, sampling_mode, planning_mode, max_meas, calibration_mode, gas_feature);
    tempTable.Properties.VariableNames = {'Map', 'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length', 'sampling_mode', 'planning_mode', 'maximal_meas', 'calibration_mode', 'gas_feature'};
    
    % Append the temporary table to the main dataTable
    dataTable = [dataTable; tempTable];
end

% Save the table to a file (e.g., CSV or MAT)
writetable(dataTable, [general_folder '/experiment_data.csv']); % Save as CSV
% save([general_folder '/experiment_data.mat'], 'dataTable');     % Save as MAT

disp('Experiment data saved.');

%% plot the result for this folder 
f = figure('Position',[-1561         656         1215         615]);
custom_color = [44,136,217]./255;
subplot(1,5,1)
plot([0 length(unique(dataTable.Map))], [0 0], 'g');
hold on;
boxplot(dataTable.x_error, dataTable.Map,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-1,10])
xlabel('Map number');
ylabel('Error (m)');
title('Error on x');
subplot(1,5,2);
plot([0 length(unique(dataTable.Map))], [0 0], 'g');
hold on;
boxplot(dataTable.y_error, dataTable.Map,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-3,4])
xlabel('Map number');
title('Error on y');
ylabel('Error (m)');
subplot(1,5,3);
boxplot(dataTable.iteration, dataTable.Map,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,40])
xlabel('Map number');
title('nbr iterations');
subplot(1,5,4)
plot([0 length(unique(dataTable.Map))], [0 0], 'g');
hold on;
boxplot(dataTable.exp_time, dataTable.Map,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
% ylim([0,300])
xlabel('Map number');
title('Experiment time');
ylabel('Time (s)');
subplot(1,5,5)
plot([0 length(unique(dataTable.Map))], [0 0], 'g');
hold on;
boxplot(dataTable.traj_length, dataTable.Map,'Colors',custom_color);
% ylim([0,50])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
%ylim([-3,4])
xlabel('Map number');
title('Traveling distance');
ylabel('Distance (m)');

figure()
plot([0 length(unique(dataTable.Map))], [0 0], 'g');
hold on;
boxplot(dataTable.maximal_meas, dataTable.Map,'Colors',custom_color);
% ylim([0,50])
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
%ylim([-3,4])
xlabel('Map number');
title('Maximal Meas');
ylabel('mV');


hold on

%% calculate success rate
success_case = 0;
for i = 1: size (dataTable.x_error,1)
    if (abs(dataTable.x_error(i))) < 1 && (abs(dataTable.y_error(i))<0.5)
        success_case = success_case+1;
    end
end
success_rate = success_case/size(dataTable.x_error,1);

annotation(f,'textbox',...
    [0.25 0.01 0.504365076491284 0.0473684201637904],...
    'String',{'Success rate:',num2str(success_rate)},...
    'LineStyle','none',...
    'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',12,...
    'FitBoxToText','off');

saveas(f,[general_folder '/test_maps_results_comparison.png']);
