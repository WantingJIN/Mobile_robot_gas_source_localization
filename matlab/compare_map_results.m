clear all; clc; close all;
dataTable = table();
%% dispatch the result to target folder
% the general folder
general_folder = '/home/wanting/Workspace/kheperaodor/plume_model_learning/Results/';
dispatched_folder = [general_folder,'test_random_source_random_start_position/baseflow'];
folder_list = dir(general_folder);
disp(folder_list)
folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'})); %,'current','101','102','103'
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

% dispatch
for i=1:exp_number

    % the name of the folder
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    % the content of the folder
    robot_list = dir(folder_name);
    robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
    robot_list = robot_list([robot_list.isdir]);
    % the IDs of the robots
    ID = arrayfun(@(struct) str2double(struct.name),robot_list);

    % skip if it's not a job folder
    if(~exist([folder_name, '/', num2str(ID(1)), '/param_log.csv'],'file')), continue; end

    % get simulation's parameter values
    params = my_text2struct([folder_name, '/', num2str(ID(1)), '/param_log.csv']);

    % remove folders with unfinished jobs
    file_list = dir([folder_name '/**']);
    if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
    entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
    %if(entropy(end,1) > 0.1 && size(entropy,1) < params.max_iter_number), rmdir(folder_name,'s'); continue; end

    % the right path of the folder
    new_path = [dispatched_folder,'/',num2str(params.map_nbr)];

    % move the folder with its friends in the right path
    if(~exist(new_path,'dir'))
        mkdir(new_path);
    end
    movefile(folder_name, new_path);
end
%

% choose the folders
folder_list = dir([dispatched_folder '/**']);
folder_list = folder_list(~ismember({folder_list.name},{'.','..', '101', '102','103'}));
folder_list = folder_list(~ismember({folder_list.folder}, dispatched_folder));
folder_list = folder_list([folder_list.isdir]);
numExperiments = length(folder_list);

%% get data
for i = 1:numExperiments
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    % multiple robots
    robot_list = dir(folder_name);
    robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
    robot_list = robot_list([robot_list.isdir]);
    ID = arrayfun(@(struct) str2num(struct.name),robot_list);
    % remove folders with unfinished jobs
    file_list = dir([folder_name '/**']);
    if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
    entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
    % ground truth and other params
    params = my_text2struct([folder_name, '/', num2str(ID(1)), '/param_log.csv']);
    map_nbr = params.map_nbr;
    % stats
    expected_a_posteriori_file = dlmread([folder_name, '/' num2str(ID(1)), '/expected_a_posteriori.csv']);
    iteration_nbr = size(expected_a_posteriori_file,1);
    expected_a_posteriori(1:2) = expected_a_posteriori_file(end,2:3);
    max_a_post = dlmread([folder_name, '/' num2str(ID(1)), '/max_a_posteriori.csv']);
    if(isnan(expected_a_posteriori(1))), expected_a_posteriori(1) = max_a_post(end,2); end
    if(isnan(expected_a_posteriori(2))), expected_a_posteriori(2) = max_a_post(end,3); end
    error(1) = params.source_position(1) - max_a_post(end,2);
    error(2) = params.source_position(2) - max_a_post(end,3);
    entropy_file = dlmread([folder_name, '/' num2str(ID(1)), '/entropy.csv']);
    entropy(1) = entropy_file(end,1);
    max_a_post_file = dlmread([folder_name, '/' num2str(ID(1)), '/max_a_posteriori.csv']);
    max_a_post(1:2) = max_a_post_file(end,2:3);
    convergence_list(1) = sqrt((max_a_post(1) - expected_a_posteriori(1))^2 + (max_a_post(2) - expected_a_posteriori(2))^2);
    % time taken for each run 
    measurements = dlmread([folder_name, '/' num2str(ID(1)), '/measurements.csv']);
    % Read all lines from the file
    lines = readlines([folder_name, '/' num2str(ID(1)), '/experiment_time.csv']);
    % Option A: assume "total time" is the last line
    totalTime = sscanf(lines(end-1), "total time (s): %f");
    % traveling_distance
    trajectory = dlmread([folder_name, '/' num2str(ID(1)), '/trajectory.csv']);
    traj_length_sum = 0;
    for iter = 2:size(trajectory,1)
        traj_length_sum = traj_length_sum + sqrt((trajectory(iter,2)-trajectory(iter-1,2))^2 + (trajectory(iter,3) - trajectory(iter-1,3))^2); 
    end
    traj_length= traj_length_sum;
    
    % strategy
    sampling_mode_idx = params.sampling_mode;
    sampling_mode = "";
    switch sampling_mode_idx
        case 0
            sampling_mode = "sense_in_motion";
        case 1
            sampling_mode = "stop_sense_go";
        case 2
            sampling_mode = "visit_sense_go";
    end

    planning_mode_idx = params.planning_strategy;
    planning_mode = "";
    switch planning_mode_idx
        case 0
            planning_mode = "greedy";
        case 5
            planning_mode = "belief_clustering";
        case 6
            planning_mode = "informative kld";
    end

    sensor_mode_idx = params.gas_sensor_type;
    sensor_mode = "";
    switch sensor_mode_idx
        case 0 
            sensor_mode = "PID";
        case 1
            sensor_mode = "MOX";
    end
    
    plume_model_idx = params.plume_model;
    plume_model = "";
    switch plume_model_idx
        case 0
            plume_model = "Gaussian";
        case 1
            plume_model = "DDPM";
        case 2
            plume_model = "PDE";
    end
    
    gas_feature_idx = params.gas_feature;
    gas_feature = "";
    switch gas_feature_idx
        case 0
            gas_feature = "absolute_value";
        case 1 
            gas_feature = "gas_hit";
        case 2
            gas_feature = "relative_value";
    end
    
    wind_uncertainty_idx = params.model_uncertainty;
    wind_estimation_idx = params.wind_estimation_method;
    wind_info_idx =  params.wind_info;
    
   
    wind_config = "";
   
    
    if (wind_info_idx == 0)
        wind_config = "wind_gt";
    end
    if (wind_info_idx == 1 && wind_estimation_idx == 2)
        wind_config = "baseflow";
    end
    if (wind_info_idx == 1 && wind_estimation_idx == 1 && wind_uncertainty_idx == 0)
        wind_config = "Gaussian_fixed_uncertainty";
    end
    if (wind_info_idx == 1 && wind_estimation_idx == 1 && wind_uncertainty_idx == 1)
        wind_config = "Gaussian_dynamic_uncertainty";
    end
            
        
            
    estimate_error=sqrt(error(1)^2+error(2)^2);
    % Create a temporary table for the current experiment
    tempTable = table(map_nbr, error(1), error(2), iteration_nbr, totalTime, traj_length, sampling_mode, planning_mode, sensor_mode, estimate_error, gas_feature, plume_model, wind_config);
    tempTable.Properties.VariableNames = {'Map', 'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length', 'sampling_mode', 'planning_mode', 'sensor_mode', 'estimate_error', 'gas_feature', 'plume_model', 'wind_config'};
    
    % Append the temporary table to the main dataTable
    dataTable = [dataTable; tempTable];
end

% Save the table to a file (e.g., CSV or MAT)
writetable(dataTable, [dispatched_folder '/experiment_data.csv']); % Save as CSV
% save([general_folder '/experiment_data.mat'], 'dataTable');     % Save as MAT

disp('Experiment data saved.');

%% plot the result for this folder 
f = figure('Position',[-1561         656         1115        600]);
ax = axes;
ax.Position = [0.13, 0.2, 0.775, 0.7];  
custom_color = [44,136,217]./255;
subplot(1,5,1)
plot([0 length(unique(dataTable.Map))], [0 0], 'g');
hold on;
boxplot(dataTable.x_error, dataTable.Map,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-10,10])
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
ylim([-4,4])
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
hold on

%% calculate success rate
success_case = 0;
for i = 1: size (dataTable.x_error,1)
    if (abs(dataTable.x_error(i))) < 1 && (abs(dataTable.y_error(i))<0.5)
        success_case = success_case+1;
    end
end
success_rate = success_case/size(dataTable.x_error,1);
% After calculating success_rate
annotation(f,'textbox',...
    [0.4, 0.01, 0.2, 0.05],...   % lower y makes it below plots
    'String',{['Success rate: ', num2str(success_rate)]},...
    'LineStyle','none',...
    'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',12,...
    'FitBoxToText','off');
xLimits = xlim;
yLimits = ylim;
% 
% % Place text at bottom center (data coordinates)
% xPos = mean(xLimits);      % middle of x-axis
% yPos = yLimits(1);         % bottom of y-axis
% offset = 0.15 * range(yLimits);  % a little above the axis line
% 
% text(xPos, yPos, 'My Annotation', ...
%     'HorizontalAlignment', 'center', ...
%     'VerticalAlignment', 'bottom', ...
%     'FontSize', 12);

saveas(f,[dispatched_folder '/test_maps_results_comparison.png']);
