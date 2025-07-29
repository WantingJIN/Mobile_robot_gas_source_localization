clear; clc; close all;
%% PARAMETROS PRINCIPALES

min_success_rate = 0.1; 

% Si no está vacío, se usa esta selección manual directamente
manual_selection = {'0021','2211','888105'};  % ← aquí defines los experimentos que quieres visualizar


filter_X = [];   % Ej: [1 2 3 4 5] o [] para 'cualquiera' (REGION_ALLOCATION)
filter_Y = [];   % Ej: [1] o [] para 'cualquiera'         (SEED)
filter_Z = [];   % Ej: [1 2 3 4 5] o [] para 'cualquiera' (PATH PLANNER)
filter_K = [2];   % Ej: [1 2] o [] para 'cualquiera'       (GAS FEATURE)

%%
boxplot_widht_subplot1 = 0.2
linspace_var_subplot1 = 0.25;

boxplot_widht_subplot2 = 0.2;
linspace_var_subplot2 = 0.27;
leyenda_dentro_subplot2 = 0;
%% Parameters
divide = 0;
color_divide = [0.875, 0.875, 0.875];


%% Configuración
% parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week18/Exp4/';
% folder_names = {'No','2','4','6','8','10','12','14','16'};
% Definir las carpetas de los 3 mapas
% Carpeta padre
parent_folder = 'C:\Users\Marc\Downloads\TFM_Results\TFM_Results\';

% Nombres de los subfolders
map_names = {'map0', 'map5', 'map9'};
map_ids = [0, 5, 9];

% Construir rutas completas automáticamente
map_folders = cellfun(@(name) fullfile(parent_folder, name), map_names, 'UniformOutput', false);

combinedTable = table();  % tabla vacía
ordered_experiments = {};

for m = 1:length(map_folders)
    current_folder = map_folders{m};
    current_map_id = map_ids(m);

    % Obtener carpetas válidas
    all_items = dir(current_folder);
    all_folders = {all_items([all_items.isdir] & ~startsWith({all_items.name}, '.')).name};

    if ~isempty(manual_selection)
        % Usar solo las carpetas seleccionadas manualmente si están presentes en esta carpeta
        folder_names = intersect(all_folders, manual_selection);
    else
        % Filtros automáticos
        valid_folders = {};
        for i = 1:length(all_folders)
            name = all_folders{i};

            if length(name) ~= 4 || ~all(isstrprop(name, 'digit'))
                continue;
            end

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

    ordered_experiments = [ordered_experiments folder_names]; %#ok<AGROW>

    % Leer datos de cada carpeta
    for i = 1:length(folder_names)
        folder_path = fullfile(current_folder, folder_names{i});
        csv_file = fullfile(folder_path, 'experiment_data.csv');

        if ~isfile(csv_file)
            warning('No se encontró: %s', csv_file);
            continue;
        end

        T = readtable(csv_file);
        T.Experiment = repmat(folder_names(i), height(T), 1);
        T.Success = abs(T.x_error) < 1 & abs(T.y_error) < 0.5;
        T.error_total = sqrt(T.x_error.^2 + T.y_error.^2);
        T.Map = repmat(current_map_id, height(T), 1);
        T.GroupLabel = repmat("Map " + current_map_id + " - " + folder_names{i}, height(T), 1);

        combinedTable = [combinedTable; T];
    end
end

%% Filtros por posición

%% Colores y nombres
blue = [44,136,217]/255;
red = [217,83,79]/255;
green = [91,183,91]/255;
orange = [255, 165, 0] / 255;  % naranja clásico
magenta = [238, 130, 238] / 255;  % También conocido como "violet"


%% Leer y unir datos de cada carpeta
% Filtrar por nombre de 4 dígitos
valid_folders = {};
for i = 1:length(all_folders)
    name = all_folders{i};
    
    if length(name) ~= 4 || ~all(isstrprop(name, 'digit'))
        continue; % saltar si no son exactamente 4 dígitos
    end
    
    X = str2double(name(1));
    Y = str2double(name(2));
    Z = str2double(name(3));
    K = str2double(name(4));
    
    if (isempty(filter_X) || ismember(X, filter_X)) && ...
       (isempty(filter_Y) || ismember(Y, filter_Y)) && ...
       (isempty(filter_Z) || ismember(Z, filter_Z)) && ...
       (isempty(filter_K) || ismember(K, filter_K))
        valid_folders{end+1} = name; %#ok<AGROW>
    end
end


%% Filtro adicional por tasa de éxito



%% Guardar resultados combinados
output_csv = fullfile('C:\Users\Marc\Downloads\results_analysis\', 'combined_experiment_data.csv');
writetable(combinedTable, output_csv);
fprintf('Datos combinados guardados en: %s\n', output_csv);

%% Colores


%% BOXPLOTS + SUCCESS RATE
%% === FIGURA 1: Métricas agregadas por EXPERIMENTO y MAPA (1 barra por experimento) ===
%% === FIGURA 1: Métricas agregadas por EXPERIMENTO y MAPA (1 barra por experimento) ===
%% BOXPLOTS + SUCCESS RATE
%% === FIGURA 1: Métricas agregadas por EXPERIMENTO y MAPA (1 barra por experimento) ===
%% === FIGURA 1: Métricas agregadas por EXPERIMENTO y MAPA (1 barra por experimento) ===
n_metrics = 4;  % Número de subplots

f1 = figure('Position',[100 100 700 490]);  % más ancho
set(f1, 'Color', 'w');

clf;
t1 = tiledlayout(2,2, 'Padding', 'compact', 'TileSpacing', 'normal');


% --- Variables base ---
map_ids_unique = unique(combinedTable.Map);
map_to_pos = containers.Map(map_ids_unique, 1:length(map_ids_unique));

unique_experiments = unique(combinedTable.Experiment, 'stable');
num_exps = numel(unique_experiments);
offsets = linspace(-linspace_var_subplot1, linspace_var_subplot1, num_exps);  % desplazamiento lateral

% Color por experimento
color_palette = {
    [91,183,91]/255;
    [44,136,217]/255;
    [238, 130, 238] / 255;
    [255, 165, 0] / 255; 
    [0.49, 0.18, 0.56];
    [0.50, 0.50, 0.50];
};
colors = containers.Map();
for i = 1:num_exps
    colors(unique_experiments{i}) = color_palette{mod(i-1, length(color_palette)) + 1};
end

% --- Métricas y funciones de agregación ---
metrics = {
    'Success Rate (%)', @(T) 100 * mean(T.Success), 'Success', [0,100];
    'Team Efficiency (%)', @(T) mean(abs(T.team_efficiency), 'omitnan'), 'Efficiency', [0,100];
    'Geometric Equity (%)', @(T) 100 * mean(T.geometric_score, 'omitnan'), 'Geometry', [0,100];
    'Informative Equity (%)', @(T) 100 * mean(T.informative_score, 'omitnan'), 'Informative', [0,100];
};

legend_handles = gobjects(num_exps, 1);  % para leyenda

for metric_idx = 1:n_metrics
    nexttile;
    xlim([0.5, 3.5]);
    hold on;

    % Fondo gris para Map 5
    if isKey(map_to_pos, 5)
        rect_width_subplot1 = 0.5;
        x_map5 = map_to_pos(5);
        patch([x_map5-rect_width_subplot1 x_map5+rect_width_subplot1 x_map5+rect_width_subplot1 x_map5-rect_width_subplot1], ...
              [0 0 100 100], color_divide, 'EdgeColor', 'none');
        x_limits = xlim;
        plot(x_limits, [0 0], 'k', 'LineWidth', 0.00000001);  % Línea eje X manual
    end

    % Una barra por experimento por mapa
    for i = 1:num_exps
        exp = unique_experiments{i};
        switch metric_idx
            case 1
                exp_color = green;     % Success Rate
            case 2
                exp_color = blue;      % Team Efficiency
            case 3
                exp_color = magenta;   % Geometric Equity
            case 4
                exp_color = orange;    % Informative Equity
        end


        for m = 1:length(map_ids_unique)
            map_id = map_ids_unique(m);
            pos_x = m + offsets(i);

            idx = strcmp(combinedTable.Experiment, exp) & combinedTable.Map == map_id;
            Tmap = combinedTable(idx, :);

            if isempty(Tmap)
                continue;
            end

            val = metrics{metric_idx, 2}(Tmap);  % función de agregación

            b = bar(pos_x, val, boxplot_widht_subplot1, 'FaceColor', exp_color, ...
                    'EdgeColor', 'k', 'LineWidth', 0.5, 'DisplayName', exp);


            if metric_idx == 1
                legend_handles(i) = b;
            end
        end
    end

    ylabel(metrics{metric_idx,1}, 'FontWeight', 'bold');
    xlabel('Map 0            Map 5            Map 9', 'FontWeight', 'bold');
    title(metrics{metric_idx,3}, 'FontWeight', 'bold');
    ylim(metrics{metric_idx,4});
    
    
    % Crear arrays vacíos para ticks y etiquetas
    all_pos_x = [];
    all_labels = {};
    
    for i = 1:num_exps
        exp = unique_experiments{i};
        for m = 1:length(map_ids_unique)
            pos_x = m + offsets(i);  % posición de cada barra
    
            all_pos_x(end+1) = pos_x; %#ok<AGROW>
            all_labels{end+1} = exp; %#ok<AGROW>
        end
    end

    % Ordenar posiciones y etiquetas para que xticks funcione sin error
    [all_pos_x_sorted, sort_idx] = sort(all_pos_x);
    all_labels_sorted = all_labels(sort_idx);
    
    xticks(all_pos_x_sorted);
    xticklabels(all_labels_sorted);
    xtickangle(45);
    
 
    set(gca, 'FontSize', 7, 'FontWeight', 'bold');
    grid on;
end



%% Subplots 2 a 6: boxplots
%% Subplots 2 a 6: boxplots por mapa
%% Subplots 2 a 6: boxplots estilo limpio agrupado por mapa
% Mapas únicos y su índice numérico
map_ids_unique = unique(combinedTable.Map);
map_to_pos = containers.Map(map_ids_unique, 1:length(map_ids_unique));

% Experimentos únicos
unique_groups = unique(combinedTable.Experiment, 'stable');
num_exps = numel(unique_groups);
offsets = linspace(-linspace_var_subplot2, linspace_var_subplot2, num_exps);


% Color por experimento
color_palette = {
    [0.00, 0.45, 0.74];   % Azul Oxford
    [0.85, 0.33, 0.10];   % Rojo granate
    [0.47, 0.67, 0.19];   % Verde oliva
    [0.93, 0.69, 0.13];   % Naranja dorado
    [0.49, 0.18, 0.56];   % Púrpura
    [0.50, 0.50, 0.50];   % Gris medio
};
colors = containers.Map();
for i = 1:num_exps
    colors(unique_groups{i}) = color_palette{mod(i-1, length(color_palette)) + 1};
end


f2 = figure('Position',[100 100 600 500]);  % más ancho
set(f2, 'Color', 'w');

t = tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'normal');

legend_handles = gobjects(num_exps,1); % solo para el primer subplot

vars = {'error_total', 'iteration', 'exp_time', 'traj_length'};
titles = {'Estimation Error', 'Iterations', 'Experimental Time', 'Trajectory Length'};
ylabels = {'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
ylims = {[-2,6], [0,35], [0,300], [0,70]};

n_metrics = 4;
n_subplots = length(vars);  % 4 métricas (success, efficiency, geom, info) + boxplots

% Subplot por variable
for k = 1:n_subplots

    nexttile;
    xlim([0.5, 3.5]);  % Fuerza que el eje X vaya exactamente de 0.5 a 3.5 → ancho total = 3
    hold on;
    
    x_map5 = 2;
    gray_color = [0.8 0.8 0.8];  % gris claro
    rect_width = 1;  % ancho del fondo (ajustable)
    
    % justo después de nexttile; antes de boxchart
    x0 = x_map5 - rect_width / 2;
    x1 = x_map5 + rect_width / 2;
    
    y_limits = ylims{k};  % usar el mismo ylim definido para la métrica
    if isempty(y_limits)
        y_limits = [0, 1];  % fallback
    end
    
    patch([x0 x1 x1 x0], [y_limits(1) y_limits(1) y_limits(2) y_limits(2)], ...
      color_divide, 'EdgeColor', 'none', 'FaceAlpha', 0.4, ...
      'HandleVisibility', 'off');



    for i = 1:num_exps
        exp = unique_groups{i};
        idx = strcmp(combinedTable.Experiment, exp);
        maps = combinedTable.Map(idx);
        values = combinedTable.(vars{k})(idx);

        % Posiciones numéricas + offset
        map_nums = arrayfun(@(m) map_to_pos(m), maps);  % mapa numérico (1, 2, 3...)
        
        % Para cada punto, calcular offset según si hay múltiples experimentos en ese mapa
        x_positions = zeros(size(map_nums));
        for j = 1:numel(map_nums)
            map_j = maps(j);
            exp_count = sum(combinedTable.Map == map_j);
            exp_in_map = unique(combinedTable.Experiment(combinedTable.Map == map_j));
            if numel(exp_in_map) == 1
                x_positions(j) = map_nums(j);  % sin offset
            else
                x_positions(j) = map_nums(j) + offsets(i);  % aplicar offset
            end
        end

        b=boxchart(x_positions, values, ...
            'BoxFaceColor', colors(exp), ...
            'BoxFaceAlpha', 0.1, ...
            'BoxWidth', boxplot_widht_subplot2, ...
            'MarkerStyle', 'o', ...
            'MarkerSize', 20*boxplot_widht_subplot2, ...
            'MarkerColor', colors(exp), ...  % ← esta línea es clave
            'LineWidth', 0.5);
        
        if k == 1
            b.DisplayName = exp; 
            legend_handles(i) = b; 
        end


    end

    % Etiquetas y estética
    set(gca, 'XTick', 1:length(map_ids_unique));
    set(gca, 'XTickLabel', string(map_ids_unique));
    xlabel('Map', 'FontWeight', 'bold');
    ylabel(ylabels{k}, 'FontWeight', 'bold');
    title(titles{k}, 'FontWeight', 'bold');
    if ~isempty(ylims{k}), ylim(ylims{k}); end
    grid on;
    box on;
    set(gca, 'FontSize', 7);

    if leyenda_dentro_subplot2 == 1
        % Leyenda
        legend(unique_groups, ...
        'Location', 'northwest', ...  
            'Color', 'w', ...
            'EdgeColor', 'none', ...
            'Box', 'on', ...
            'FontWeight', 'bold');
    end
end

% Solo crea la leyenda si todos los handles son válidos
if leyenda_dentro_subplot2 == 0
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
%%
%% === FIGURA EXTRA: Tiempo medio por robot ===
% Crear una copia para no modificar el original
adjustedTable = combinedTable;

% Obtener IDs de experimento
exp_ids = string(adjustedTable.Experiment);

% Detectar los que no empiezan con '0' → multirobot
is_multirobot = ~startsWith(exp_ids, '0');

% Multiplicar tiempo por 3 sólo para multirobot
adjustedTable.exp_time(is_multirobot) = adjustedTable.exp_time(is_multirobot) * 3;

% Crear la figura igual que 'Experimental Time' pero con los tiempos modificados
f3 = figure('Position',[100 100 363 273]);
set(f3, 'Color', 'w');
t3 = tiledlayout(1,1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
hold on;

xlim([0.5, 3.5]);

% Fondo gris para Map 5
x_map5 = 2;
rect_width = 1;
x0 = x_map5 - rect_width / 2;
x1 = x_map5 + rect_width / 2;
p = patch([x0 x1 x1 x0], [0 0 9000 300], color_divide, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.4);
set(p, 'HandleVisibility', 'off');  % ← esto es clave

% Dibujar boxcharts
for i = 1:num_exps
    exp = unique_groups{i};
    idx = strcmp(adjustedTable.Experiment, exp);
    maps = adjustedTable.Map(idx);
    values = adjustedTable.exp_time(idx);

    map_nums = arrayfun(@(m) map_to_pos(m), maps);

    x_positions = zeros(size(map_nums));
    for j = 1:numel(map_nums)
        map_j = maps(j);
        exp_in_map = unique(adjustedTable.Experiment(adjustedTable.Map == map_j));
        if numel(exp_in_map) == 1
            x_positions(j) = map_nums(j);
        else
            x_positions(j) = map_nums(j) + offsets(i);
        end
    end

    boxchart(x_positions, values, ...
        'BoxFaceColor', colors(exp), ...
        'BoxFaceAlpha', 0.1, ...
        'BoxWidth', boxplot_widht_subplot2, ...
        'MarkerStyle', 'o', ...
        'MarkerSize', 20*boxplot_widht_subplot2, ...
        'MarkerColor', colors(exp), ...
        'LineWidth', 0.5);
end

% Estética
set(gca, 'XTick', 1:length(map_ids_unique));
set(gca, 'XTickLabel', string(map_ids_unique));
xlabel('Map', 'FontWeight', 'bold');
ylabel('Time (s)', 'FontWeight', 'bold');
title('Average Time Per Robot', 'FontWeight', 'bold');
ylim([0, 500]);  % mismo ylim que "Experimental Time"
grid on;
box on;
set(gca, 'FontSize', 8);

% Leyenda
lgd3 = legend(unique_groups, ...
    'Orientation', 'horizontal', ...
    'FontWeight', 'bold', ...
    'EdgeColor', 'none', ...
    'Box', 'on', ...
    'Color', 'w');
lgd3.Layout.Tile = 'south';

