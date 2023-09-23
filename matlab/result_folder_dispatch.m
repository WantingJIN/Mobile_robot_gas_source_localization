close all; clc; clear all;

% the general folder
general_folder = '/home/durancea/plume_model_learning/Results';
folder_list = dir(general_folder);
folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'}));
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

%%
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
    % remove folders with unfinished jobs
    file_list = dir([folder_name '/**']);
    if(sum(ismember({file_list.name},{'position_history.csv'})) == 0), rmdir(folder_name,'s'); continue; end
    
    % get simulation's parameter values
    params = my_text2struct([folder_name, '/', num2str(ID(1)), '/param_log.csv']);

    % the right path of the folder
    new_path = [general_folder '/results_stopsense_PID/' num2str(params.map_nbr)];

    % move the folder with its friends in the right path
    if(~exist(new_path,'dir'))
        mkdir(new_path);
    end
    movefile(folder_name, new_path);
end