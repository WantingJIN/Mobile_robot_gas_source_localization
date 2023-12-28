clear all; clc; %close all;

%% dispatch
% the general folder
general_folder = '/home/durancea/plume_model_learning/Results/bin_fixed_corrected_map_1_penalty2.5';
folder_list = dir(general_folder);
folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'}));
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

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
    if(entropy(end,1) > 0.1 && size(entropy,1) < params.max_iter_number), rmdir(folder_name,'s'); continue; end

    % the right path of the folder
    new_path = [general_folder '/' num2str(params.map_nbr)];

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
    map_nbr_list(i,1) = params.map_nbr;
    % stats
    expected_a_posteriori_file = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/expected_a_posteriori.csv']);
    iteration_nbr_list(i) = size(expected_a_posteriori_file,1);
    expected_a_posteriori(i,1:2) = expected_a_posteriori_file(end,2:3);
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
    % time taken for each run 
    measurement = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/measurements.csv']);
    % exp_time_list(i) = measurement(end,4);
    
    exp_time_file = readtable([folder_name, '/', num2str(ID(1)), '/experiment_time.csv']);
    nav_time = exp_time_file(2,2);
    comp_time = exp_time_file(3,2);
    exp_time_list(i) = nav_time.Var2 + comp_time.Var2; 
    % traveling_distance
    trajectory = dlmread([folder_list(i).folder, '/', folder_list(i).name, '/' num2str(ID(1)), '/trajectory.csv']);
    traj_length_sum = 0;
    for iter = 2:size(trajectory,1)
        traj_length_sum = traj_length_sum + sqrt((trajectory(iter,2)-trajectory(iter-1,2))^2 + (trajectory(iter,3) - trajectory(iter-1,3))^2); 
    end
    traj_length_list(i) = traj_length_sum;
end

%% boxplots
% f = figure('units','normalized','outerposition',[0 0 1 1]);
% subplot(5,1,1)
% plot([0 length(unique(map_nbr_list))], [0 0], 'g');
% hold on;
% boxplot(error_list(:,1), map_nbr_list);
% grid on;
% title('error on x');
% subplot(5,1,2);
% plot([0 length(unique(map_nbr_list))], [0 0], 'g');
% hold on;
% boxplot(error_list(:,2), map_nbr_list);
% grid on;
% title('error on y');
% subplot(5,1,3);
% boxplot(iteration_nbr_list, map_nbr_list);
% grid on;
% title('nbr iterations');
% subplot(5,1,4);
% boxplot(entropy_list, map_nbr_list);
% grid on;
% title('final entropy');
% subplot(5,1,5);
% boxplot(convergence_list, map_nbr_list);
% grid on;
% title('convergence');
% 
% %% save
% saveas(f,[general_folder '/test_maps_results_comparison_all.png']);


%% plot for thesis
f = figure('Position',[-1561         656         1215         615]);
custom_color = [44,136,217]./255;
subplot(1,5,1)
plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
boxplot(error_list(:,1), map_nbr_list,'Colors',custom_color);
error_x = error_list(:,1);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-1,10])
xlabel('Map number');
ylabel('Error (m)');
title('Error on x');

subplot(1,5,2);
plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
boxplot(error_list(:,2), map_nbr_list,'Colors',custom_color);
error_y = error_list(:,2);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([-3,4])
xlabel('Map number');
title('Error on y');
ylabel('Error (m)');

subplot(1,5,3);
boxplot(iteration_nbr_list, map_nbr_list,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
ylim([0,60])
xlabel('Map number');
title('nbr iterations');

subplot(1,5,4)
plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
boxplot(exp_time_list, map_nbr_list,'Colors',custom_color);
lines = findobj(gcf, 'type', 'line', 'Tag', 'Median');
set(lines, 'Color', 'r');
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
%ylim([-3,4])
xlabel('Map number');
title('Experiment time');
ylabel('Time (s)');

subplot(1,5,5)
plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
boxplot(traj_length_list, map_nbr_list,'Colors',custom_color);
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
for i = 1: size (error_x,1)
    if (abs(error_x(i))) < 1 && (abs(error_y(i))<0.5)
        success_case = success_case+1;
    end
end
success_rate = success_case/size(error_x,1)

annotation(f,'textbox',...
    [0.25 0.01 0.504365076491284 0.0473684201637904],...
    'String',{'Success rate:',num2str(success_rate)},...
    'LineStyle','none',...
    'HorizontalAlignment','center',...
    'FontWeight','bold',...
    'FontSize',12,...
    'FitBoxToText','off');

saveas(f,[general_folder '/test_maps_results_comparison.png']);
%% save the statistical result for t-test
save([general_folder '/error_statistics.mat'], 'error_x', 'error_y', 'iteration_nbr_list', 'success_rate');

