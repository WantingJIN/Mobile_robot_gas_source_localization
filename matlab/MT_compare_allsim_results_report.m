clear; clc; close all;
%% PARAMETROS PRINCIPALES

min_success_rate = 0.1; 

% === Filtros selección manual ===
manual_selection = {'1231','99911', '99913'};  % <<<<<< Aquí pones los que quieras ver

% === Filtros automaticos ===
filter_X = [];   % Ej: [1 2 3 4 5] o [] para 'cualquiera' (REGION_ALLOCATION)
filter_Y = [];   % Ej: [1] o [] para 'cualquiera'         (SEED)
filter_Z = [];   % Ej: [1 2 3 4 5] o [] para 'cualquiera' (PATH PLANNER)
filter_K = [1];   % Ej: [1 2] o [] para 'cualquiera'       (GAS FEATURE)

map = 0;

boxplot_widht = 0.01;

linspace_var = 0.4;

leyenda_dentro = 0;
%% Parameters
divide = 0;
color_divide = [0.85, 0.85, 0.85];

%% Configuración
% parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week18/Exp4/';
% folder_names = {'No','2','4','6','8','10','12','14','16'};
%parent_folder = sprintf('C:\Users\Marc\Downloads\TFM_Results\TFM_Results\map%d', map);
parent_folder = ['C:\Users\Marc\Downloads\TFM_Results\TFM_Results\map', num2str(map)];

%% Filtros por posición
% Obtener carpetas válidas
all_items = dir(parent_folder);
all_folders = {all_items([all_items.isdir] & ~startsWith({all_items.name}, '.')).name};

% Filtrar por nombre de 4 dígitos
valid_folders = {};
folder_names = {};

if ~isempty(manual_selection)
    % Si hay selección manual, se usa directamente
    folder_names = manual_selection;
else
    % Filtros automáticos como antes
    all_items = dir(parent_folder);
    all_folders = {all_items([all_items.isdir] & ~startsWith({all_items.name}, '.')).name};

    valid_folders = {};
    for i = 1:length(all_folders)
        name = all_folders{i};

%         if length(name) ~= 4 || ~all(isstrprop(name, 'digit'))
%             continue;
%         end

        X = str2double(name(1));
        Y = str2double(name(2));
        Z = str2double(name(3));
        K = str2double(name(4));

        if (isempty(filter_X) || ismember(X, filter_X)) && ...
           (isempty(filter_Y) || ismember(Y, filter_Y)) && ...
           (isempty(filter_Z) || ismember(Z, filter_Z)) && ...
           (isempty(filter_K) || ismember(K, filter_K))
            valid_folders{end+1} = name;
        end
    end

    folder_names = valid_folders;
end

ordered_experiments = folder_names;


combinedTable = table();

%% Colores y nombres
blue = [44,136,217]/255;
red = [217,83,79]/255;
green = [91,183,91]/255;
orange = [255, 165, 0] / 255;  % naranja clásico
magenta = [238, 130, 238] / 255;  % También conocido como "violet"


%% Leer y unir datos de cada carpeta
for i = 1:length(folder_names)
    folder_path = fullfile(parent_folder, folder_names{i});
    csv_file = fullfile(folder_path, 'experiment_data.csv');

    if ~isfile(csv_file)
        warning('No se encontró: %s', csv_file);
        continue;
    end

    T = readtable(csv_file);
    T.Experiment = repmat(folder_names(i), height(T), 1); % etiqueta
    T.Success = abs(T.x_error) < 1 & abs(T.y_error) < 0.5; % tasa éxito
    T.error_total = sqrt(T.x_error.^2 + T.y_error.^2);
    combinedTable = [combinedTable; T];
end

%% Filtro adicional por tasa de éxito de cada experimento
valid_experiments = {};
for i = 1:length(folder_names)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, folder_names{i}), :);
    if ~isempty(exp_data)
        success_rate = sum(exp_data.Success) / height(exp_data);
        if success_rate >= min_success_rate
            valid_experiments{end+1} = folder_names{i};
        end
    end
end

% Aplicar el filtro al conjunto de datos y actualizar la lista de experimentos
combinedTable = combinedTable(ismember(combinedTable.Experiment, valid_experiments), :);
ordered_experiments = valid_experiments;


%% Filtro adicional por tasa de éxito

%% Guardar resultados combinados
output_csv = fullfile(parent_folder, 'combined_experiment_data.csv');
writetable(combinedTable, output_csv);
fprintf('Datos combinados guardados en: %s\n', output_csv);

%%
%% Figura 2: Boxplots estilo limpio con offsets
vars = {'error_total', 'iteration', 'exp_time', 'traj_length'};
titles = {'Total Error', 'Iterations', 'Time', 'Distance'};
ylabels = {'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
ylims = {[-3,10], [0,40], [0,300], [0,150]};
n_metrics = 4;
n_subplots = length(vars);  % 4 métricas (success, efficiency, geom, info) + boxplots

%% BOXPLOTS + SUCCESS RATE
f1 = figure('Position',[50 50 1750 300]);  % más ancho
set(f1, 'Color', 'w');


%% Subplot 1: Success rate por experimento
figure(1)
subplot(1, n_metrics, 1);

hold on;

% Experimentos ordenados
%ordered_experiments = {'A','B'};
success_rates = zeros(length(ordered_experiments), 1);

% Calcular tasas de éxito
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        success_rates(i) = sum(exp_data.Success) / height(exp_data);
    else
        success_rates(i) = NaN;
    end
end

% === Fondo gris en las primeras columnas ===
ylim_success = [0, 100];
for j = 1:divide
    x0 = j - 0.5;
    x1 = j + 0.5;
    patch([x0 x1 x1 x0], [ylim_success(1) ylim_success(1) ylim_success(2) ylim_success(2)], ...
          color_divide, 'EdgeColor', 'none');
end

% === Gráfico de barras con índice numérico ===
bar(1:length(success_rates), success_rates * 100, 'FaceColor', green, 'EdgeColor', 'none');

% Ajustes de ejes y etiquetas
ylim(ylim_success);
ylabel('Success Rate (%)', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
t = title('Success', 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;

% xticks(1:length(ordered_experiments));
% xticklabels(ordered_experiments);
% set(gca, 'TickLabelInterpreter', 'none');  % <<--- Desactiva LaTeX
% xtickangle(45);

xticks([]);

grid on;
set(gca, 'FontSize', 11, 'FontWeight', 'bold');

%% Team efficiency

team_efficiency_vals = zeros(length(ordered_experiments), 1);
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        team_efficiency_vals(i) = mean(abs(exp_data.team_efficiency), 'omitnan');
    else
        team_efficiency_vals(i) = NaN;
    end
end

figure(1)
subplot(1, n_metrics, 2);
hold on;

ylim_eff = [0, 100];
for j = 1:divide
    x0 = j - 0.5;
    x1 = j + 0.5;
    patch([x0 x1 x1 x0], [ylim_eff(1) ylim_eff(1) ylim_eff(2) ylim_eff(2)], ...
          color_divide, 'EdgeColor', 'none');
end

bar(1:length(team_efficiency_vals), team_efficiency_vals, 'FaceColor', blue, 'EdgeColor', 'none');
t = title('Efficiency ', 'FontWeight', 'bold');
ylabel('Team Efficiency (%)', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;

% xticks(1:length(ordered_experiments));
% xticklabels(ordered_experiments);
% xtickangle(45);
% ylim(ylim_eff);

xticks([]);

grid on;
set(gca, 'FontSize', 11, 'FontWeight', 'bold');



%% Subplot 2: Geometric Score promedio por experimento
figure(1)
subplot(1, n_metrics, 3);
hold on;

geom_scores = zeros(length(ordered_experiments), 1);

for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        geom_scores(i) = mean(exp_data.geometric_score, 'omitnan');
    else
        geom_scores(i) = NaN;
    end
end

for j = 1:divide
    x0 = j - 0.5; x1 = j + 0.5;
    patch([x0 x1 x1 x0], [0 0 100 100], color_divide, 'EdgeColor', 'none');
end

bar(1:length(geom_scores), geom_scores*100, 'FaceColor', magenta,  'EdgeColor', 'none');
ylim([0 100]);
ylabel('Geometric Equity (%)', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
t = title('Geometry', 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;
% xticks(1:length(ordered_experiments));
% xticklabels(ordered_experiments);
% xtickangle(45);

xticks([]);

set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;

%% Subplot 3: Informative Score promedio por experimento
figure(1)
subplot(1, n_metrics, 4);
hold on;

info_scores = zeros(length(ordered_experiments), 1);

for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        info_scores(i) = mean(exp_data.informative_score, 'omitnan');
    else
        info_scores(i) = NaN;
    end
end

for j = 1:divide
    x0 = j - 0.5; x1 = j + 0.5;
    patch([x0 x1 x1 x0], [0 0 100 100], color_divide, 'EdgeColor', 'none');
end

bar(1:length(info_scores), info_scores*100, 'FaceColor', orange,  'EdgeColor', 'none');
ylim([0 100]);
ylabel('Informative Equity (%)', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
t = title('Informative', 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;
% xticks(1:length(ordered_experiments));
% xticklabels(ordered_experiments);
% xtickangle(45);

xticks([]);

set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;


%% Figura independiente: Success Rate por experimento
f_success = figure('Position',[100, 100, 800, 400]);
set(f_success, 'Color', 'w');

% Calcular tasas de éxito
success_rates = zeros(length(ordered_experiments), 1);
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        success_rates(i) = sum(exp_data.Success) / height(exp_data);
    else
        success_rates(i) = NaN;
    end
end

% Fondo gris opcional (si se quiere dividir)
ylim_success = [0, 100];
for j = 1:divide
    x0 = j - 0.5;
    x1 = j + 0.5;
    patch([x0 x1 x1 x0], [ylim_success(1) ylim_success(1) ylim_success(2) ylim_success(2)], ...
          color_divide, 'EdgeColor', 'none');
end

% Gráfico de barras
bar(1:length(success_rates), success_rates * 100, ...
    'FaceColor', green, ...
    'EdgeColor', 'none');

% Ajustes visuales
ylim(ylim_success);
ylabel('Success Rate (%)', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
title('Success Rate per Experiment', 'FontWeight', 'bold');
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);

set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;

% --------- Efficiency Plot ---------
f_efficiency = figure('Position', [100, 100, 800, 400]);
set(f_efficiency, 'Color', 'w');

efficiency_scores = zeros(length(ordered_experiments), 1);
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        efficiency_scores(i) = mean(abs(exp_data.team_efficiency));
    else
        efficiency_scores(i) = NaN;
    end
end

for j = 1:divide
    x0 = j - 0.5;
    x1 = j + 0.5;
    patch([x0 x1 x1 x0], [0 0 100 100], color_divide, 'EdgeColor', 'none');
end

bar(1:length(efficiency_scores), efficiency_scores, ...
    'FaceColor', blue, ...
    'EdgeColor', 'none');

ylim([0, 100]);
ylabel('Efficiency (%)', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
title('Efficiency per Experiment', 'FontWeight', 'bold');
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;


% --------- Informative Score Plot ---------
f_info = figure('Position', [100, 100, 800, 400]);
set(f_info, 'Color', 'w');

info_scores = zeros(length(ordered_experiments), 1);
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        info_scores(i) = mean(exp_data.informative_score*100);
    else
        info_scores(i) = NaN;
    end
end

for j = 1:divide
    x0 = j - 0.5;
    x1 = j + 0.5;
    patch([x0 x1 x1 x0], [0 0 100 100], color_divide, 'EdgeColor', 'none');
end

bar(1:length(info_scores), info_scores, ...
    'FaceColor', orange, ...
    'EdgeColor', 'none');

ylim([0, 100]);
ylabel('Informative Score', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
title('Informative Score per Experiment', 'FontWeight', 'bold');
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;


% --------- Geometric Score Plot ---------
f_geom = figure('Position', [100, 100, 800, 400]);
set(f_geom, 'Color', 'w');

geom_scores = zeros(length(ordered_experiments), 1);
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        geom_scores(i) = mean(exp_data.geometric_score*100);
    else
        geom_scores(i) = NaN;
    end
end

for j = 1:divide
    x0 = j - 0.5;
    x1 = j + 0.5;
    patch([x0 x1 x1 x0], [0 0 100 100], color_divide, 'EdgeColor', 'none');
end

bar(1:length(geom_scores), geom_scores, ...
    'FaceColor', magenta, ...
    'EdgeColor', 'none');

ylim([0, 100]);
ylabel('Geometric Score', 'FontWeight', 'bold');
xlabel(sprintf('Map %d', map), 'FontWeight', 'bold');
title('Geometric Score per Experiment', 'FontWeight', 'bold');
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;

%% Subplots 2 a 6: boxplots


%% Figura 2: Boxplots estilo limpio con offsets
f2 = figure('Position',[100 100 1750 300]); set(f2, 'Color', 'w');
t = tiledlayout(2,2, 'Padding', 'compact', 'TileSpacing', 'compact');

unique_groups = ordered_experiments;
num_exps = numel(unique_groups);
offsets = linspace(-linspace_var, linspace_var, num_exps);

color_palette = {
    [0.00, 0.45, 0.74]; [0.85, 0.33, 0.10]; [0.47, 0.67, 0.19];
    [0.93, 0.69, 0.13]; [0.49, 0.18, 0.56]; [0.50, 0.50, 0.50];
};
colors = containers.Map();
for i = 1:num_exps
    colors(unique_groups{i}) = color_palette{mod(i-1, length(color_palette)) + 1};
end

map_ids_unique = [map];  % Añade esta línea
map_to_pos = containers.Map(map_ids_unique, 1:length(map_ids_unique));


% Dentro del for k=1:n_subplots
legend_handles = gobjects(num_exps,1); % solo para el primer subplot

for k = 1:n_subplots
    ax = nexttile; hold on;
    x_map = 1;
    xlim([0.5, 1.5]);

    for i = 1:num_exps
        exp = unique_groups{i};
        idx = strcmp(combinedTable.Experiment, exp);
        values = combinedTable.(vars{k})(idx);
        pos = x_map + offsets(i);

        b = boxchart(repmat(pos, size(values)), values, ...
            'BoxFaceColor', colors(exp), ...
            'BoxFaceAlpha', 0.1, ...
            'BoxWidth', boxplot_widht, ...
            'MarkerStyle', 'o', ...
            'MarkerColor', colors(exp), ...
            'LineWidth', 0.5);

        if k == 1
            b.DisplayName = exp; 
            legend_handles(i) = b; 
        end
    end

    ylabel(ylabels{k}, 'FontWeight', 'bold');
    title(titles{k}, 'FontWeight', 'bold');
    ylim(ylims{k}); grid on; box on;
    set(gca, 'XTick', 1, 'XTickLabel', {'Map ' + string(map)}, 'FontSize', 11, 'FontWeight', 'bold');

    if leyenda_dentro == 1
        % Leyenda
        legend(unique_groups, ...
        'Location', 'northwest', ...  
            'Color', 'w', ...
            'EdgeColor', 'none', ...
            'Box', 'on', ...
            'FontWeight', 'bold');
    end

end












% de aqui en adelante dentro del loop es para generar figuras
% individuales
% === Generar figuras individuales por métrica ===
metric_names = {'Total Error', 'Iterations', 'Time', 'Distance'};
metric_ylabels = { 'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
metric_vars = {'error_total', 'iteration', 'exp_time', 'traj_length'};
metric_ylims = { [-3, 10], [0, 40], [0, 300], [0, 150]};

for m = 1:length(metric_vars)
    f_ind = figure('Position', [300 300 450 300]);
    set(f_ind, 'Color', 'w');
    hold on;

    % Boxplots por experimento
    offsets = linspace(-linspace_var, linspace_var, num_exps);
    for i = 1:num_exps
        exp = unique_groups{i};
        idx = strcmp(combinedTable.Experiment, exp);
        val = combinedTable.(metric_vars{m})(idx);
        pos = 1 + offsets(i);
        boxchart(repmat(pos, size(val)), val, ...
            'BoxFaceColor', colors(exp), ...
            'BoxFaceAlpha', 0.1, ...
            'BoxWidth', boxplot_widht, ...
            'MarkerStyle', 'o', ...
            'MarkerColor', colors(exp), ...
            'LineWidth', 0.5);
    end

    % Etiquetas de los experimentos debajo del boxplot
    set(gca, ...
        'XTick', 1 + offsets, ...
        'XTickLabel', unique_groups, ...
        'FontSize', 11, ...
        'FontWeight', 'bold');

    xtickangle(45);
    ylabel(metric_ylabels{m}, 'FontWeight', 'bold');
    title(sprintf('%s - Map %d', metric_names{m}, map), 'FontWeight', 'bold');

    xlim([0.5, 1.5]);
    ylim(metric_ylims{m});  % <- correcto: usa el ylim de la métrica correspondiente

    grid on; box on;
end
















% Solo crea la leyenda si todos los handles son válidos
if leyenda_dentro == 0
    if all(isgraphics(legend_handles))
        lgd = legend(legend_handles, unique_groups, ...
            'Orientation', 'horizontal', ...
            'FontWeight', 'bold', ...
            'EdgeColor', 'none', ...
            'Box', 'on', ...
            'Color', 'w');

    % Ponemos la leyenda fuera del tiledlayout para que no interfiera con el padding
    lgd.Layout.Tile = 'south';  % Esto la ubica justo debajo, centrada horizontalmente
    
else
    warning('Algunos handles para leyenda no son válidos');
end
end

t.Padding = 'compact';
t.TileSpacing = 'compact';



sgtitle('', 'FontSize', 16, 'FontWeight', 'bold');


%% Guardar figura
saveas(f1, fullfile(parent_folder, 'combined_metrics_boxplot.png'));
saveas(f2, fullfile(parent_folder, 'combined_results_boxplot.png'));
