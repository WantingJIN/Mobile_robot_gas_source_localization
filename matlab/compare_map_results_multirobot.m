clear all; clc; close all;

dataTable = table();

% Paths
general_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/';
dispatched_folder = [general_folder, 'test'];
folder_list = dir(general_folder);

% Exclude non-experiment folders
folder_list = folder_list(~ismember({folder_list.name}, {'.','..','current','test'}));
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

% Dispatch folders
for i = 1:exp_number
    folder_name = fullfile(folder_list(i).folder, folder_list(i).name);
    robot_list = dir(folder_name);
    robot_list = robot_list(~ismember({robot_list.name}, {'.','..'}));
    robot_list = robot_list([robot_list.isdir]);

    ID = arrayfun(@(s) str2double(s.name), robot_list);

    if isempty(ID) || ~exist(fullfile(folder_name, num2str(ID(1)), 'param_log.csv'), 'file')
        fprintf('Saltando carpeta (sin robots o sin param_log): %s\n', folder_name);
        continue;
    end

    % Get params
    params = my_text2struct(fullfile(folder_name, num2str(ID(1)), 'param_log.csv'));

    % Remove folders with unfinished jobs
    file_list = dir([folder_name '/**']);
    if ~any(ismember({file_list.name}, {'position_history.csv'}))
        rmdir(folder_name, 's');
        continue;
    end

    % Dispatch path
    new_path = fullfile(dispatched_folder, num2str(params.map_nbr));
    if ~exist(new_path, 'dir')
        mkdir(new_path);
    end

    movefile(folder_name, new_path);
end

% Collect experiment folders
folder_list = dir([dispatched_folder '/**']);
folder_list = folder_list(~ismember({folder_list.name}, {'.','..'}));
folder_list = folder_list([folder_list.isdir]);
folder_list = folder_list(~strcmp({folder_list.folder}, dispatched_folder));
numExperiments = length(folder_list);

%% Collect data
for i = 1:numExperiments
    folder_name = fullfile(folder_list(i).folder, folder_list(i).name);
    robot_list = dir(folder_name);
    robot_list = robot_list(~ismember({robot_list.name}, {'.','..'}));
    robot_list = robot_list([robot_list.isdir]);

    % Initialize metrics
    pos_errors = [];
    traj_lengths = [];
    times = [];
    entropies = [];
    iterations = [];

    for r = 1:length(robot_list)
        robot_id = robot_list(r).name;
        robot_path = fullfile(folder_name, robot_id);

        if ~exist(fullfile(robot_path, 'param_log.csv'), 'file')
            continue;
        end

        % Params and files
        params = my_text2struct(fullfile(robot_path, 'param_log.csv'));
        map_nbr = params.map_nbr;

        try
            expected_a_posteriori_file = dlmread(fullfile(robot_path, 'expected_a_posteriori.csv'));
            expected_a_posteriori = expected_a_posteriori_file(end,2:3);

            max_a_post_file = dlmread(fullfile(robot_path, 'max_a_posteriori.csv'));
            max_a_post = max_a_post_file(end,2:3);

            entropy_file = dlmread(fullfile(robot_path, 'entropy.csv'));
            entropy = entropy_file(end,1);

            measurements = dlmread(fullfile(robot_path, 'measurements.csv'));
            exp_time = measurements(end,4);

            trajectory = dlmread(fullfile(robot_path, 'trajectory.csv'));
            traj_length = sum(sqrt(sum(diff(trajectory(:,2:3)).^2, 2)));

            % Position error
            err = reshape(params.source_position, 1, []) - reshape(expected_a_posteriori, 1, []);
            pos_errors(end+1,:) = err;
            traj_lengths(end+1) = traj_length;
            times(end+1) = exp_time;
            entropies(end+1) = entropy;
            iterations(end+1) = size(expected_a_posteriori_file, 1);
        catch
            warning('Datos incompletos para robot %s en %s. Saltando.', robot_id, folder_name);
            continue;
        end
    end

    if isempty(pos_errors)
        continue;
    end

    % Estimaciones promedio
    mean_error = mean(pos_errors, 1);
    mean_estimate_error = norm(mean_error);
    mean_traj = mean(traj_lengths);
    mean_time = mean(times);
    mean_iter = mean(iterations);

    % Modo de operación (se toma de primer robot)
    sampling_mode = ["sense in motion", "stop sense go", "visit sense go"];
    planning_mode = ["greedy", "", "", "", "", "belief clustering", "informative kld"];
    sensor_mode = ["PID", "MOX"];

    s_mode = sampling_mode(params.sampling_mode+1);
    p_mode = planning_mode(params.planning_strategy+1);
    sens_mode = sensor_mode(params.gas_sensor_type+1);

    % Guardar fila
    tempTable = table(map_nbr, mean_error(1), mean_error(2), mean_iter, mean_time, mean_traj, ...
                      s_mode, p_mode, sens_mode, mean_estimate_error, ...
                      'VariableNames', {'Map', 'x_error', 'y_error', 'iteration', ...
                                        'exp_time', 'traj_length', 'sampling_mode', ...
                                        'planning_mode', 'sensor_mode', 'estimate_error'});
    dataTable = [dataTable; tempTable];
end

% Guardar tabla
writetable(dataTable, [dispatched_folder '/experiment_data.csv']);
disp('Datos guardados.');

%% Gráficos
f = figure('Position',[-1561, 656, 2015, 1215]);
custom_color = [44,136,217]/255;

disp(dataTable.Properties.VariableNames);


subplot(1,5,1)
plot([0 length(unique(dataTable.Map))], [0 0], 'g'); hold on;
boxplot(dataTable.x_error, dataTable.Map, 'Colors', custom_color);
set(findobj(gca,'type','line'),'linew',1.2)
grid on; ylim([-1,10])
xlabel('Map number'); ylabel('Error (m)'); title('Error on x');

subplot(1,5,2)
plot([0 length(unique(dataTable.Map))], [0 0], 'g'); hold on;
boxplot(dataTable.y_error, dataTable.Map, 'Colors', custom_color);
set(findobj(gca,'type','line'),'linew',1.2)
grid on; ylim([-3,4])
xlabel('Map number'); ylabel('Error (m)'); title('Error on y');

subplot(1,5,3)
boxplot(dataTable.iteration, dataTable.Map, 'Colors', custom_color);
set(findobj(gca,'type','line'),'linew',1.2)
grid on; ylim([0,40])
xlabel('Map number'); title('Nbr iterations');

subplot(1,5,4)
boxplot(dataTable.exp_time, dataTable.Map, 'Colors', custom_color);
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
xlabel('Map number'); ylabel('Time (s)'); title('Experiment time');

subplot(1,5,5)
boxplot(dataTable.traj_length, dataTable.Map, 'Colors', custom_color);
set(findobj(gca,'type','line'),'linew',1.2)
grid on;
xlabel('Map number'); ylabel('Distance (m)'); title('Traveling distance');

% Success rate
success_case = sum(abs(dataTable.x_error) < 1 & abs(dataTable.y_error) < 0.5);
success_rate = success_case / height(dataTable);
annotation(f, 'textbox', [0.25 0.01 0.504 0.047], ...
    'String', {'Success rate:', num2str(success_rate)}, ...
    'LineStyle','none', 'HorizontalAlignment','center', ...
    'FontWeight','bold', 'FontSize',12);

saveas(f, [dispatched_folder '/test_maps_results_comparison.png']);
