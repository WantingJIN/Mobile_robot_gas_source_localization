clear all; clc; close all;
dataTable = table();

%% Rutas de carpetas
general_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/';
dispatched_folder = [general_folder, 'test'];
folder_list = dir(general_folder);

% Filtrar carpetas válidas
folder_list = folder_list(~ismember({folder_list.name},{'.','..','current','101','102','103'}));
folder_list = folder_list([folder_list.isdir]);
exp_number = length(folder_list);

%% Despachar resultados
for i = 1:exp_number
    folder_name = fullfile(folder_list(i).folder, folder_list(i).name);
    
    % Obtener subcarpeta válida
    subfolders = dir(folder_name);
    subfolders = subfolders([subfolders.isdir]); % Solo carpetas
    subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'})); % Excluir '.' y '..'
    
    if isempty(subfolders)
        continue; % Ignorar si no hay subcarpetas
    end
    
    if length(subfolders) > 1
        warning('Más de una subcarpeta encontrada en "%s". Usando la primera.', folder_name);
    end
    
    % Seleccionar la primera subcarpeta
    selected_subfolder = fullfile(subfolders(1).folder, subfolders(1).name);
    
    if ~exist(fullfile(selected_subfolder, 'param_log.csv'), 'file')
        continue; % Ignorar si no existe el archivo param_log.csv
    end

    params = my_text2struct(fullfile(selected_subfolder, 'param_log.csv'));
    
    file_list = dir([folder_name '/**']);
    if ~any(ismember({file_list.name}, {'position_history.csv'}))
        rmdir(folder_name, 's'); 
        continue;
    end

    entropy = dlmread(fullfile(selected_subfolder, 'entropy.csv'));
    
    new_path = fullfile(dispatched_folder, num2str(params.map_nbr));
    if ~exist(new_path, 'dir')
        mkdir(new_path);
    end
    movefile(folder_name, new_path);
end

%% Obtener data
folder_list = dir([dispatched_folder '/**']);
folder_list = folder_list(~ismember({folder_list.name},{'.','..'}));
folder_list = folder_list([folder_list.isdir]);
numExperiments = length(folder_list);

for i = 1:numExperiments
    folder_name = fullfile(folder_list(i).folder, folder_list(i).name);
    
    % Obtener subcarpeta válida
    subfolders = dir(folder_name);
    subfolders = subfolders([subfolders.isdir]); % Solo carpetas
    subfolders = subfolders(~ismember({subfolders.name}, {'.', '..'})); % Excluir '.' y '..'
    
    if isempty(subfolders)
        continue; % Ignorar si no hay subcarpetas
    end
    
    if length(subfolders) > 1
        warning('Más de una subcarpeta encontrada en "%s". Usando la primera.', folder_name);
    end
    
    % Seleccionar la primera subcarpeta
    selected_subfolder = fullfile(subfolders(1).folder, subfolders(1).name);
    
    if ~exist(fullfile(selected_subfolder, 'param_log.csv'), 'file')
        continue; % Ignorar si no existe el archivo param_log.csv
    end

    entropy = dlmread(fullfile(selected_subfolder, 'entropy.csv'));
    params = my_text2struct(fullfile(selected_subfolder, 'param_log.csv'));

    map_nbr = params.map_nbr;
    expected_a_posteriori_file = dlmread(fullfile(selected_subfolder, 'expected_a_posteriori.csv'));
    iteration_nbr = size(expected_a_posteriori_file, 1);
    expected_a_posteriori = expected_a_posteriori_file(end,2:3);
    
    max_a_post_file = dlmread(fullfile(selected_subfolder, 'max_a_posteriori.csv'));
    max_a_post = max_a_post_file(end,2:3);

    expected_a_posteriori(isnan(expected_a_posteriori)) = max_a_post(isnan(expected_a_posteriori));
    
    error = params.source_position(1:2) - expected_a_posteriori;
    entropy_val = entropy(end,1);
    convergence_list = sqrt(sum((max_a_post - expected_a_posteriori).^2));
    
    measurements = dlmread(fullfile(selected_subfolder, 'measurements.csv'));
    exp_time = measurements(end,4);

    trajectory = dlmread(fullfile(selected_subfolder, 'trajectory.csv'));
    traj_length = sum(sqrt(sum(diff(trajectory(:,2:3)).^2, 2)));

    % Estrategias
    sampling_modes = ["sense in motion", "stop sense go", "visit sense go"];
    planning_modes = ["greedy", "", "", "", "", "belief clustering", "informative kld"];
    sensor_modes = ["PID", "MOX"];

    sampling_mode = sampling_modes(params.sampling_mode + 1);
    planning_mode = planning_modes(params.planning_strategy + 1);
    sensor_mode = sensor_modes(params.gas_sensor_type + 1);

    estimate_error = norm(error);

    tempTable = table(map_nbr, error(1), error(2), iteration_nbr, exp_time, traj_length, sampling_mode, planning_mode, sensor_mode, estimate_error);
    tempTable.Properties.VariableNames = {'Map', 'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length', 'sampling_mode', 'planning_mode', 'sensor_mode', 'estimate_error'};
    dataTable = [dataTable; tempTable];
end

writetable(dataTable, [dispatched_folder '/experiment_data.csv']);

disp('Experiment data saved.');

%% Gráfica de resultados
f = figure('Position',[100 100 1800 600]);

% Paleta de colores
blue = [44,136,217]/255;
red = [217,83,79]/255;
green = [91,183,91]/255;

vars = {'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length'};
titles = {'Error en X', 'Error en Y', 'Número de iteraciones', 'Tiempo de experimento', 'Distancia recorrida'};
ylabels = {'Error (m)', 'Error (m)', 'Iteraciones', 'Tiempo (s)', 'Distancia (m)'};
ylims = {[-1,10], [-3,4], [0,40], [], []};

for k = 1:5
    subplot(1,5,k);
    boxplot(dataTable.(vars{k}), dataTable.Map, ...
        'Colors', blue, 'Symbol', 'k+', 'Widths', 0.5, 'Whisker', 1.5);
    hold on;
    yline(0, '--g', 'LineWidth', 1.2);
    lines = findobj(gca, 'type', 'line', 'Tag', 'Median');
    set(lines, 'Color', red);
    set(findobj(gca,'type','line'),'LineWidth',1.2);
    grid on;
    xlabel('Mapa', 'FontWeight', 'bold');
    ylabel(ylabels{k}, 'FontWeight', 'bold');
    title(titles{k}, 'FontWeight', 'bold');
    if ~isempty(ylims{k}), ylim(ylims{k}); end
    set(gca, 'FontSize', 11, 'FontWeight', 'bold');
end

sgtitle('Comparación de resultados por mapa', 'FontSize', 16, 'FontWeight', 'bold');

%% Tasa de éxito
success_case = sum(abs(dataTable.x_error) < 1 & abs(dataTable.y_error) < 0.5);
success_rate = success_case / height(dataTable);

annotation(f,'textbox', ...
    [0.25 0.01 0.504 0.05], ...
    'String',{sprintf('Tasa de éxito: %.2f%%', success_rate*100)}, ...
    'LineStyle','none', ...
    'HorizontalAlignment','center', ...
    'FontWeight','bold', ...
    'FontSize',14, ...
    'Color', green);

% Guardar figura
exportgraphics(f, [dispatched_folder '/test_maps_results_comparison.png'], 'Resolution', 300);
