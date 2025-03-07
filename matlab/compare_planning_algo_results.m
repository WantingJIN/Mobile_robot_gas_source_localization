% Compares the GSL performance for difference gas release amounts
% Plots distance to true source location along y and x and the number of
% iterations for convergence

clear all; clc; close all;
dataTable = table();
%% dispatch
% the general folder
general_folder = '/home/wjin/Workspace/continuous_STE/Results/';
dispatched_folder = [general_folder,'IROS_2025_results/map10'];
folder_list = dir(general_folder);
disp(folder_list)
folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'})); %,'current','101','102','103'
folder_list = folder_list([folder_list.isdir]);
numExperiments = length(folder_list);

use_iteration = 0;

% dispatch
for i=1:numExperiments
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
%     if(entropy(end,1) > 0.1 && size(entropy,1) < params.max_iter_number), rmdir(folder_name,'s'); continue; end
    sub_folder_name = '';
    if params.planning_strategy == 5
        sub_folder_name = ['belief_clustering', num2str(params.cluster_ratio)];
    end
    if params.planning_strategy == 8
        sub_folder_name = 'A_star';
    end
    if params.planning_strategy == 6
        sub_folder_name = 'informative_KLD';
    end
    if params.planning_strategy == 10
        sub_folder_name = 'Dijkstra_Fix';
        sub_folder_name = [sub_folder_name, num2str(params.score_sigma)];
    end
    if params.planning_strategy == 11
        sub_folder_name = 'Dijkstra_ADP';
        sub_folder_name = [sub_folder_name, num2str(params.weighting_entropy_ratio)];
    end
    if params.planning_strategy == 10 || params.planning_strategy == 11
        if params.score_fun == 0
            sub_folder_name = [sub_folder_name, '_Ratio'];
        end
        if params.score_fun == 1
            sub_folder_name = [sub_folder_name, '_Log'];
        end
        if params.score_fun == 2
            sub_folder_name = [sub_folder_name, '_Sigmoid'];
        end
        if params.score_fun == 3
            sub_folder_name = [sub_folder_name, '_Sum'];
        end
        if params.score_fun == 4
            sub_folder_name = [sub_folder_name, '_Exp'];
        end
        if params.info_feature == 0
            sub_folder_name = [sub_folder_name, '_variance'];
        end
        if params.info_feature == 1
            sub_folder_name = [sub_folder_name, '_mean'];
        end
        if params.info_feature == 2
            sub_folder_name = [sub_folder_name, '_pdf'];
        end
        if params.info_sum == 0
            sub_folder_name = [sub_folder_name, '_linear_acc'];
        end
        if params.info_sum == 1
            sub_folder_name = [sub_folder_name, '_nonlinear_acc'];
        end
    end
   

    % the right path of the folder
    new_path = [dispatched_folder '/' sub_folder_name];

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
for i=1:numExperiments
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
    expected_a_posteriori_file = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/expected_a_posteriori.csv']);
    iteration_nbr = size(expected_a_posteriori_file,1);
    expected_a_posteriori(1:2) = expected_a_posteriori_file(end,2:3);
    
    % Time test
    exp_time_file = readtable([folder_name, '/', num2str(ID(1)), '/experiment_time.csv']);
    % nav_time = exp_time_file(2,2);
    % comp_time = exp_time_file(3,2);
    % exp_time_list(i) = nav_time.Var2 + comp_time.Var2; 
    %exp_time = exp_time_file(1,5).Var5; 
    exp_time = exp_time_file(3,4).Var4; 
    total_time = exp_time_file(4,4).Var4;
    nor_cmp_time = exp_time/iteration_nbr;
    % error on X and Y (col 5 and 6)
    max_a_post = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/max_a_posteriori.csv']);
    if(isnan(expected_a_posteriori(1))), expected_a_posteriori(1) = max_a_post(end,2); end
    if(isnan(expected_a_posteriori(2))), expected_a_posteriori(2) = max_a_post(end,3); end
    error(1) = params.source_position(1) - expected_a_posteriori(1);
    error(2) = params.source_position(2) - expected_a_posteriori(2);
    entropy = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/entropy.csv']);
    entropy(1) = entropy(end,1);
    
    max_a_post(1:2) = max_a_post(end,2:3);
    %convergence(1) = sqrt((max_a_post(1) - expected_a_posteriori(i,1))^2 + (max_a_post_list(i,2) - expected_a_posteriori(i,2))^2);
    
    % traveling_distance
    trajectory = dlmread([folder_name, '/' num2str(ID(1)), '/trajectory.csv']);
    traj_length_sum = 0;
    for iter = 2:size(trajectory,1)
        traj_length_sum = traj_length_sum + sqrt((trajectory(iter,2)-trajectory(iter-1,2))^2 + (trajectory(iter,3) - trajectory(iter-1,3))^2); 
    end
    traj_length= traj_length_sum;

    % strategy
    gas_feature_idx = params.gas_feature;
    gas_hit_method_idx = params.gas_hit_method;
    gas_feature = "";
    switch gas_feature_idx
        case 0
            gas_feature = "gas concentration";
        case 1
            if gas_hit_method_idx == 5
                gas_feature = "fixed gas hit";
            end
            if gas_hit_method_idx == 4
                gas_feature = "adaptive gas hit";
            end
        case 2
            gas_feature = "gas measurement rank";
    end
    % path planning strategy
    path_planning_idx = params.planning_strategy;
    planning_algo = "";
    switch path_planning_idx
        case 5
            planning_algo = append("clustering", num2str(params.cluster_ratio));
        case 8
            planning_algo = "A star";
        case 10
            planning_algo = "Dijkstra_FIX";
            if params.score_fun == 0
                planning_algo = append(planning_algo, "_Ratio");
            end
            if params.score_fun == 1
                planning_algo = append(planning_algo, "_Log");
            end
            if params.score_fun == 2
                planning_algo = append(planning_algo, "_Sigmoid");
            end
            if params.score_fun == 3
                planning_algo = append(planning_algo, "_Sum");
            end
            if params.score_fun == 4
                planning_algo = append(planning_algo, "_Exp");
            end
            if params.info_sum == 0
                planning_algo = append(planning_algo, "_linear_sum");
            end
            if params.info_sum == 1
                planning_algo = append(planning_algo, "_nonlinear_sum");
            end
            planning_algo = append(planning_algo, num2str(params.score_sigma)); 
        case 6
            planning_algo = "informative KLD";
        case 11
            planning_algo = "Dijkstra_ADP";
            if params.score_fun == 0
                planning_algo = append(planning_algo, "_Ratio");
            end
            if params.score_fun == 1
                planning_algo = append(planning_algo, "_Log");
            end
            if params.score_fun == 2
                planning_algo = append(planning_algo, "_Sigmoid");
            end
            if params.score_fun == 3
                planning_algo = append(planning_algo, "_Sum");
            end
            if params.score_fun == 4
                planning_algo = append(planning_algo, "_Exp");
            end
            if params.info_sum == 0
                planning_algo = append(planning_algo, "_linear_sum");
            end
            if params.info_sum == 1
                planning_algo = append(planning_algo, "_nonlinear_sum");
            end
            if params.info_feature == 0
                planning_algo = append(planning_algo, "_variance");
            end
            if params.info_feature == 1
                planning_algo = append(planning_algo, "_mean");
            end
            if params.info_feature == 2
                planning_algo = append(planning_algo, "_pdf");
            end
            planning_algo = append(planning_algo, num2str(params.weighting_entropy_ratio)); 
    end
            
    estimate_error=sqrt(error(1)^2+error(2)^2);
    % Create a temporary table for the current experiment
    tempTable = table(map_nbr, error(1), error(2), iteration_nbr, exp_time, estimate_error, gas_feature, planning_algo, traj_length, total_time, nor_cmp_time);
    tempTable.Properties.VariableNames = {'Map', 'x_error', 'y_error', 'iteration', 'exp_time', 'estimate_error', 'gas_feature', 'planning_algo','traj_length', 'total_time', 'nor_cmp_time'};
    
    % Append the temporary table to the main dataTable
    dataTable = [dataTable; tempTable];
end

% Save the table to a file (e.g., CSV or MAT)
writetable(dataTable, [dispatched_folder '/experiment_data.csv']); % Save as CSV
% save([general_folder '/experiment_data.mat'], 'dataTable');     % Save as MAT

disp('Experiment data saved.');

%% plot for thesis
f = figure('Position',[-1561         656         715         615]);
custom_color = [44,136,217]./255;
subplot(3,2,1)
plot([0 length(unique(dataTable.planning_algo))], [0 0], 'g');
hold on;
boxplot(dataTable.x_error, dataTable.planning_algo,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-3,10])
xlabel('Planning Algo');
ylabel('Error (m)');
title('Error on x');
subplot(3,2,2);
% sensor_type_char = cellfun(@num2str, sensor_type, 'UniformOutput', false);
plot([0 length(unique(dataTable.planning_algo))], [0 0], 'g');
hold on;
boxplot(dataTable.y_error, dataTable.planning_algo,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-2,2])
xlabel('Planning Algo');
title('Error on y');
ylabel('Error (m)');
subplot(3,2,3);
boxplot(dataTable.iteration, dataTable.planning_algo,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,30])
xlabel('Planning Algo');
ylabel('nbr iterations');
title('Total nbr iterations'); 

subplot(3,2,4);
boxplot(dataTable.traj_length, dataTable.planning_algo,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
%     ylim([0,max(iteration_nbr_list)+1])
xlabel('Planning Algo');
ylabel('m');
ylim([0,50])
title('Trajctory Length');

subplot(3,2,5);
boxplot(dataTable.nor_cmp_time, dataTable.planning_algo,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
%     ylim([0,max(iteration_nbr_list)+1])
xlabel('Planning Algo');
ylabel('s');
title('Computation Time');


figure()
boxplot(dataTable.total_time, dataTable.planning_algo,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
%     ylim([0,max(iteration_nbr_list)+1])
xlabel('Planning Algo');
ylabel('s');
title('Experimental Time');

%% calculate success rate
success_case = 0;
for i = 1: size(dataTable.x_error,1)
    if (abs(dataTable.x_error(i))) < 1 && (abs(dataTable.y_error(i))<0.5)
        success_case = success_case+1;
    end
end
success_rate = success_case/size(dataTable.x_error,1)

annotation(f,'textbox',...
    [0.25 0.01 0.504365076491284 0.0473684201637904],...
    'String',{'Success rate:',num2str(success_rate)},...
    'LineStyle','none',...
    'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',12,...
    'FitBoxToText','off');

saveas(f,[dispatched_folder '/path_planning_algo_comparison.png']);

Mean_first = mean(dataTable.total_time(1:10))
variance_first = std(dataTable.total_time(1:10))

Mean_first = mean(dataTable.total_time(11:20))
variance_first = std(dataTable.total_time(11:20))