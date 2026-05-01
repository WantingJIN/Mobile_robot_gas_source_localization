clear all; clc; close all;

%% Inicialización
dataTable = table();

% Rutas
general_folder = '/home/wjin/Workspace/Multi-robot_GSL/Results';
dispatched_folder = fullfile(general_folder, 'DARS_2026/absolute_calibrated_sensor_measurement_sharing');
% dispatched_folder_map = fullfile(general_folder, 'Multi-GSL/REGION_GROWING_2D_BALANCED_center/9');
folder_list = dir(general_folder);
folder_list = folder_list([folder_list.isdir]); % Solo carpetas

% Filtrar carpetas no deseadas
excluded = {'.','..','current','101','102','103'};
folder_list = folder_list(~ismember({folder_list.name}, excluded));
exp_number = length(folder_list);

%% Despachar resultados por mapa
for i = 1:exp_number
    folder_name = fullfile(folder_list(i).folder, folder_list(i).name);
    
    % Buscar subcarpeta válida
    subfolders = dir(folder_name);
    subfolders = subfolders([subfolders.isdir] & ~ismember({subfolders.name}, {'.','..'}));
    if isempty(subfolders), continue; end
    if length(subfolders) > 1
        warning('Más de una subcarpeta en "%s". Usando la primera.', folder_name);
    end
    
    selected_subfolder = fullfile(subfolders(1).folder, subfolders(1).name);
    if ~exist(fullfile(selected_subfolder, 'param_log.csv'), 'file'), continue; end

    % Cargar parámetros
    params = my_text2struct(fullfile(selected_subfolder, 'param_log.csv'));

    % Verificar existencia de datos
    file_list = dir([folder_name '/**']);
    if ~any(ismember({file_list.name}, {'position_history.csv'}))
        rmdir(folder_name, 's'); 
        continue;
    end
    
    % Mover carpeta a la ruta organizada
    new_path = fullfile(dispatched_folder, num2str(params.map_nbr));
    if ~exist(new_path, 'dir'), mkdir(new_path); end
    movefile(folder_name, new_path);
end

%% Extraer información y crear tabla
folder_list = dir([dispatched_folder '/**']);
folder_list = folder_list([folder_list.isdir] & ~ismember({folder_list.name}, {'.','..'}));
numExperiments = length(folder_list);

for i = 1:numExperiments
    folder_name = fullfile(folder_list(i).folder, folder_list(i).name);
    subfolders = dir(folder_name);
    subfolders = subfolders([subfolders.isdir] & ~ismember({subfolders.name}, {'.','..'}));
    if isempty(subfolders), continue; end
    if length(subfolders) > 1
        warning('Más de una subcarpeta en "%s". Usando la primera.', folder_name);
    end

    % Carpeta principal para obtener métricas principales
    selected_subfolder = fullfile(subfolders(1).folder, subfolders(1).name);
    if ~exist(fullfile(selected_subfolder, 'param_log.csv'), 'file'), continue; end

    % Cargar datos principales
    params = my_text2struct(fullfile(selected_subfolder, 'param_log.csv'));
    entropy = dlmread(fullfile(selected_subfolder, 'entropy.csv'));
    expected_a_posteriori_file = dlmread(fullfile(selected_subfolder, 'expected_a_posteriori.csv'));
    max_a_post_file = dlmread(fullfile(selected_subfolder, 'max_a_posteriori.csv'));
    % measurements = dlmread(fullfile(selected_subfolder, 'measurements.csv'));
    sim_time = dlmread(fullfile(selected_subfolder, 'simulation_time.csv'));  % tamaño 1x1

    % Cálculos principales
    %iteration_nbr = size(expected_a_posteriori_file, 1);
    iteration_nbr = expected_a_posteriori_file(end, 1);
    expected_a_posteriori = expected_a_posteriori_file(end,2:3);
    max_a_post = max_a_post_file(end,2:3);
    expected_a_posteriori(isnan(expected_a_posteriori)) = max_a_post(isnan(expected_a_posteriori));
    error = params.source_position(1:2) - expected_a_posteriori;
    estimate_error = norm(error);
    entropy_val = entropy(end,1);
    exp_time = sim_time;

    % Calcular distancia total usando todas las subcarpetas (101, 102, 103)
    traj_length = 0;
    for robot_id = [101, 102, 103]
        robot_folder = fullfile(folder_name, num2str(robot_id));
%         if exist(fullfile(robot_folder, 'trajectory.csv'), 'file')
%         %if exist(fullfile(robot_folder, 'cumulative_path.csv'), 'file')
%             trajectory = dlmread(fullfile(robot_folder, 'trajectory.csv'));
%             %trajectory = dlmread(fullfile(robot_folder, 'cumulative_path.csv'));
%             traj_length = traj_length + sum(sqrt(sum(diff(trajectory(:,2:3)).^2, 2)));
%         end
        position_file = fullfile(robot_folder, 'whole_position_history.csv');
        if exist(position_file, 'file')
            positions = dlmread(position_file); % columnas: [x, y]
            traj_length = traj_length + sum(sqrt(sum(diff(positions).^2, 2)));
        end
    end
    
    % === Team Efficiency ===
    stop_time_total = 0;
    for robot_id = [101, 102, 103]
        stop_file = fullfile(folder_name, num2str(robot_id), 'stop_time.csv');
        if exist(stop_file, 'file')
            stop_data = dlmread(stop_file); % columna única
            stop_time_total = stop_time_total + sum(stop_data);
        end
    end
    team_efficiency = (1 - (stop_time_total/3) / sim_time) * 100;


     % === Balance scores ===
    H_geometric_all = [];
    H_informative_all = [];

    for robot_id = [101, 102, 103]
        robot_folder = fullfile(folder_name, num2str(robot_id));

        geo_file = fullfile(robot_folder, 'cluster_sizes_geometric.csv');
        info_file = fullfile(robot_folder, 'cluster_sizes_informative.csv');

        if exist(geo_file, 'file') && exist(info_file, 'file')
            data_cluster_geometric = readmatrix(geo_file);
            data_cluster_informative = readmatrix(info_file);

            % Inicializar vectores para H fila por fila
            H_geometric_rows = nan(size(data_cluster_geometric,1),1);
            H_informative_rows = nan(size(data_cluster_informative,1),1);

            for idx = 1:size(data_cluster_geometric,1)
                % Geometric
                row_geo = data_cluster_geometric(idx,:);
                total_geo = sum(row_geo);
                if total_geo > 0
                    p_geo = row_geo / total_geo;
                    p_geo(p_geo == 0) = [];
                    H_val_geo = -sum(p_geo .* log2(p_geo));
                    H_geometric_rows(idx) = H_val_geo / log2(3);
                else
                    H_geometric_rows(idx) = 0;
                end

                % Informative
                row_info = data_cluster_informative(idx,:);
                total_info = sum(row_info);
                if total_info > 0
                    p_info = row_info / total_info;
                    p_info(p_info == 0) = [];
                    H_val_info = -sum(p_info .* log2(p_info));
                    H_informative_rows(idx) = H_val_info / log2(3);
                else
                    H_informative_rows(idx) = 0;
                end
            end

            % Agregar todos los valores para esta simulación (todos los robots)
            H_geometric_all = [H_geometric_all; H_geometric_rows];
            H_informative_all = [H_informative_all; H_informative_rows];
        end
    end

    % Promediar todos los valores de H para esta simulación
    H_geometric = mean(H_geometric_all, 'omitnan');
    H_informative = mean(H_informative_all, 'omitnan');



    % Etiquetas
    sampling_modes = ["sense in motion", "stop sense go", "visit sense go"];
    planning_modes = ["greedy","", "", "", "", "belief clustering", "informative kld"];
    sensor_modes = ["PID", "MOX","NOISY_MOX", "SLOW_MOX","VERY_SLOW_MOX"];
    
    sampling_mode = sampling_modes(params.sampling_mode + 1);
    planning_mode = 13;
    %planning_mode = planning_modes(params.planning_strategy + 1);
    sensor_mode = sensor_modes(params.gas_sensor_type + 1);

    % Guardar fila de resultados
    %tempTable = table(params.map_nbr, error(1), error(2), iteration_nbr, exp_time, traj_length, sampling_mode, planning_mode, sensor_mode, estimate_error);
    %tempTable.Properties.VariableNames = {'Map', 'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length', 'sampling_mode', 'planning_mode', 'sensor_mode', 'estimate_error'};
    tempTable = table(params.map_nbr, error(1), error(2), iteration_nbr, exp_time, traj_length, ...
        sampling_mode, planning_mode, sensor_mode, estimate_error, ...
        H_geometric, H_informative, team_efficiency);

    tempTable.Properties.VariableNames = {'Map', 'x_error', 'y_error', 'iteration', 'exp_time', ...
        'traj_length', 'sampling_mode', 'planning_mode', 'sensor_mode', 'estimate_error', ...
        'geometric_score', 'informative_score', 'team_efficiency'};

    dataTable = [dataTable; tempTable];
end

% Guardar CSV
csv_path = fullfile(dispatched_folder, 'experiment_data.csv');
writetable(dataTable, csv_path);
disp(['Datos exportados a: ', csv_path]);

%% Gráfica de comparación
%f = figure('Position',[100 100 1800 950], 'Color', 'w');  % Fondo blanco
f = figure('Visible', 'off', 'Position',[100 100 1800 950], 'Color', 'w');


% Paleta de colores
blue = [44,136,217]/255;
red = [217,83,79]/255;
green = [91,183,91]/255;

vars = {'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length'};
titles = {'X error', 'Y error', 'Iterations', 'Time', 'Distance'};
ylabels = {'Error (m)', 'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
ylims = {[-1,10], [-3,4], [0,40], [0,300], [0,150]};

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
    xlabel('Map', 'FontWeight', 'bold');
    ylabel(ylabels{k}, 'FontWeight', 'bold');
    title(titles{k}, 'FontWeight', 'bold');
    if ~isempty(ylims{k}), ylim(ylims{k}); end
    set(gca, 'FontSize', 11, 'FontWeight', 'bold');
end

sgtitle(' ', 'FontSize', 16, 'FontWeight', 'bold');

% %% Tasa de éxito
% success_case = sum(abs(dataTable.x_error) < 1 & abs(dataTable.y_error) < 0.5);
% success_rate = success_case / height(dataTable);

% annotation(f,'textbox', [0.25 0.01 0.504 0.05], ...
%     'String',{sprintf('Success rate: %.2f%%', success_rate*100)}, ...
%     'LineStyle','none', ...
%     'HorizontalAlignment','center', ...
%     'FontWeight','bold', ...
%     'FontSize',14, ...
%     'Color', green);
% 
% % Guardar figura
% saveas(f, fullfile(dispatched_folder, 'test_maps_results_comparison.png'));
% 
% %% Tasa de balanceo
% 
% % === Balance Score promedio ===
% avg_geometric = mean(dataTable.geometric_score, 'omitnan');
% avg_informative = mean(dataTable.informative_score, 'omitnan');
% 
% annotation(f,'textbox', [0.75 0.01 0.25 0.05], ...
%     'String',{sprintf('Avg Geometric: %.2f | Avg Informative: %.2f', avg_geometric, avg_informative)}, ...
%     'LineStyle','none', ...
%     'HorizontalAlignment','center', ...
%     'FontWeight','bold', ...
%     'FontSize',14, ...
%     'Color', blue);
% 
% === Métricas globales ===
success_case = sum(abs(dataTable.x_error) < 1 & abs(dataTable.y_error) < 0.5);
success_rate = success_case / height(dataTable);
avg_geometric = mean(dataTable.geometric_score, 'omitnan');
avg_informative = mean(dataTable.informative_score, 'omitnan');
avg_team_eff = mean(dataTable.team_efficiency, 'omitnan');

% Mostrar métricas en la parte inferior de la figura
metrics_text = {
    sprintf('\\bfAvg Success rate: \\rm%.2f%%', success_rate*100), ...
    sprintf('\\bfAvg Geometric Score: \\rm%.2f%%', avg_geometric*100), ...
    sprintf('\\bfAvg Informative Score: \\rm%.2f%%', avg_informative*100), ...
    sprintf('\\bfAvg Team Efficiency: \\rm%.2f%%', avg_team_eff)
};


annotation(f, 'textbox', [0 0.015 1 0.04], ...
    'String', strjoin(metrics_text, '     |     '), ...
    'FontSize', 14, ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'LineStyle', 'none', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', [1, 1, 0.6], ...  % Subrayado amarillo suave
    'Margin', -2, ...
    'Color', [0.1 0.1 0.1]);


%% Guardar figura
saveas(f, fullfile(dispatched_folder, 'results_boxplot.png'));