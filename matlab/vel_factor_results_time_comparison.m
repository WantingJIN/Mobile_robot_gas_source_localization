% Compares the GSL execution time in absolute experiment time and number of
% iterations for different constant velocities


clear all; clc; %close all;

%% dispatch
% the general folder
general_folder = '/home/durancea/plume_model_learning/Results/robot_vel_influence/time_comp_d0.1p10';
folder_list = dir(general_folder);
disp(folder_list)
folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'})); %,'current','101','102','103'
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

use_time = 0;
use_cst_velocity=1;

if(use_cst_velocity)
    x_axis_title = 'Constant linear velocity (mm/s)';
else
    x_axis_title = x_axis_title;
end
%% dispatch
for i=1:exp_number
    
    disp(i)
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
    new_path = [general_folder '/' num2str(params.vel_factor)];

    % move the folder with its friends in the right path
    if(~exist(new_path,'dir'))
        mkdir(new_path);
    end
    movefile(folder_name, new_path);
end
%%

% choose the folders
folder_list = dir([general_folder '/**']);
folder_list = folder_list(~ismember({folder_list.name},{'.','..', '101', '102','103'}));
folder_list = folder_list(~ismember({folder_list.folder}, general_folder));
folder_list = folder_list([folder_list.isdir]);
number = length(folder_list);

%% get data
for i=1:number
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    
    disp('folder_name')
    disp(folder_name)
    
    % multiple robots
    robot_list = dir(folder_name)
    robot_list = robot_list(~ismember({robot_list.name},{'.','..'}))
    robot_list = robot_list([robot_list.isdir])
    ID = arrayfun(@(struct) str2num(struct.name),robot_list);
    % remove folders with unfinished jobs
    file_list = dir([folder_name '/**']);
    % if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
    entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
  
    % ground truth and other params
    params = my_text2struct([folder_name, '/', num2str(ID(1)), '/param_log.csv']); 
    if (use_cst_velocity) 
        vel_factor_list(i,1) = params.vel_factor*813;
    else
        vel_factor_list(i,1) = params.vel_factor;
    end
    % stats
    expected_a_posteriori_file = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/expected_a_posteriori.csv']);
    iteration_nbr_list(i) = size(expected_a_posteriori_file,1);
    expected_a_posteriori(i,1:2) = expected_a_posteriori_file(end,2:3);
    
    
        exp_time_file = readtable([folder_name, '/', num2str(ID(1)), '/experiment_time.csv']);
        nav_time = exp_time_file(2,2);
        comp_time = exp_time_file(3,2);
        exp_time_list(i) = nav_time.Var2 + comp_time.Var2; 
   
    
    % error on X and Y (col 5 and 6)
    max_a_post = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/max_a_posteriori.csv']);
    if(isnan(expected_a_posteriori(i,1))), expected_a_posteriori(i,1) = max_a_post(end,2); end
    if(isnan(expected_a_posteriori(i,2))), expected_a_posteriori(i,2) = max_a_post(end,3); end
    error_list(i,1) = params.source_position(1) - expected_a_posteriori(i,1);
    error_list(i,2) = params.source_position(2) - expected_a_posteriori(i,2);
    entropy = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/entropy.csv']);
    entropy_list(i,1) = entropy(end,1);
    
    max_a_post_list(i,1:2) = max_a_post(end,2:3);
    convergence_list(i,1) = sqrt((max_a_post_list(i,1) - expected_a_posteriori(i,1))^2 + (max_a_post_list(i,2) - expected_a_posteriori(i,2))^2);
end


%% plot for thesis without time

%% plot for thesis
f = figure('Position',[-1561         656         715         615]);
custom_color = [44,136,217]./255;

subplot(1,2,1)
% plot([0 length(unique(vel_factor_list))], [0 0], 'g');
hold on;
boxplot(exp_time_list, vel_factor_list,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,max(exp_time_list)+1])
xlabel(x_axis_title);
ylabel('Time (s)');
title('Total experiment time');

subplot(1,2,2);
% plot([0 length(unique(vel_factor_list))], [0 0], 'g');
hold on;
boxplot(iteration_nbr_list, vel_factor_list,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,max(iteration_nbr_list)+1])
xlabel(x_axis_title);
ylabel('nb iterations');
title('nb iterations');

saveas(f,[general_folder '/test_maps_results_comparison.png']);



