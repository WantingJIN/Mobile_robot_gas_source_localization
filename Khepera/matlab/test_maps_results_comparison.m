clear all; clc; close all;

%% dispatch
% the general folder
general_folder = '/home/wanting/Workspace/kheperaodor/plume_model_learning/Khepera/Results';
folder_list = dir(general_folder);
folder_list = folder_list(~ismember({folder_list.name},{'.','..','bad','101','102','103'}));
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

for i=1:exp_number
    % the name of the folder
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    folder_name_k4 = [folder_list(i).folder, '/', folder_list(i).name, '/k4'];
    folder_name_pc = [folder_list(i).folder, '/', folder_list(i).name, '/pc'];
   
     % skip if it's not a job folder
    if(~exist([folder_name_pc '/param_log.csv'],'file')), continue; end
    
    % remove folders with unfinished jobs
%     file_list = dir([folder_name '/**']);
%     if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
%     entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
%     if(entropy(end,1) > 0.1 && size(entropy,1) < 200), rmdir(folder_name,'s'); continue; end
    
    % get simulation's parameter values
    params = my_text2struct([folder_name_pc, '/param_log.csv']);

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
folder_list = folder_list(~ismember({folder_list.name},{'.','..', '101', '102','103', 'pc', 'k4'}));
folder_list = folder_list(~ismember({folder_list.folder}, general_folder));
folder_list = folder_list(~ismember({folder_list.folder}, [general_folder '/bad']));
folder_list = folder_list([folder_list.isdir]);
number = length(folder_list);

%% get data
for i=1:number
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    folder_name_k4 = [folder_list(i).folder, '/', folder_list(i).name, '/k4'];
    folder_name_pc = [folder_list(i).folder, '/', folder_list(i).name, '/pc'];
    % remove folders with unfinished jobs
%     file_list = dir([folder_name '/**']);
%     if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
%     entropy = dlmread([folder_name, '/', num2str(ID(1)), '/entropy.csv']);
%     if(entropy(end,1) > 0.1 && size(entropy,1) < 200), rmdir(folder_name,'s'); continue; end
    % ground truth and other params
    params = my_text2struct([folder_name_pc, '/param_log.csv']);
    map_nbr_list(i,1) = params.map_nbr;
    % stats
    expected_a_posteriori_file = dlmread([folder_name_pc, '/expected_a_posteriori.csv']);
    iteration_nbr_list(i) = size(expected_a_posteriori_file,1);
    expected_a_posteriori(i,1:2) = expected_a_posteriori_file(end,2:3);
    % error on X and Y (col 5 and 6)
    max_a_post = dlmread([folder_name_pc, '/max_a_posteriori.csv']);
    if(isnan(expected_a_posteriori(i,1))), expected_a_posteriori(i,1) = max_a_post(end,2); end
    if(isnan(expected_a_posteriori(i,2))), expected_a_posteriori(i,2) = max_a_post(end,3); end
    error_list(i,1) = params.source_position(1) - expected_a_posteriori(i,1);
    error_list(i,2) = params.source_position(2) - expected_a_posteriori(i,2);
    entropy = dlmread([folder_name_pc, '/entropy.csv']);
    entropy_list(i,1) = entropy(end,1);
    
    max_a_post_list(i,1:2) = max_a_post(end,2:3);
    convergence_list(i,1) = sqrt((max_a_post_list(i,1) - expected_a_posteriori(i,1))^2 + (max_a_post_list(i,2) - expected_a_posteriori(i,2))^2);
end

%% boxplots
f = figure('units','normalized','outerposition',[0 0 1 1]);
subplot(5,1,1)
plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
boxplot(error_list(:,1), map_nbr_list);
grid on;
title('error on x');
subplot(5,1,2);
plot([0 length(unique(map_nbr_list))], [0 0], 'g');
hold on;
boxplot(error_list(:,2), map_nbr_list);
grid on;
title('error on y');
subplot(5,1,3);
boxplot(iteration_nbr_list, map_nbr_list);
grid on;
title('nbr iterations');
subplot(5,1,4);
boxplot(entropy_list, map_nbr_list);
grid on;
title('final entropy');
subplot(5,1,5);
boxplot(convergence_list, map_nbr_list);
grid on;
title('convergence');

%% save
saveas(f,[general_folder '/test_maps_results_comparison.png']);
