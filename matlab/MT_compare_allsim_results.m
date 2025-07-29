clear; clc; close all;

%% Parameters
divide = 0;
color_divide = [0.85, 0.85, 0.85];

%% Configuración
% parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week18/Exp4/';
% folder_names = {'No','2','4','6','8','10','12','14','16'};
parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/results_analysis/RELATIVE_VALUE/';

%folder_names = {'AVS_G', 'AVS_B', 'RVS_G', 'RVS_B','AV_G','AV_B', 'RV_G', 'RV_B'};
%folder_names = {'AVS', 'RVS', 'AV', 'RV'};
%folder_names = {'AVS_G','AVS_B','AVS_DF','AVS_DA','AV_G','AV_B','AV_DF','AV_DA','AV_I'};
%folder_names = {'4KAV_I','8KAV_I','10KAV_I','12KAV_I','14KAV_I','16KAV_I','18KAV_I','NOKAV_I'};
%folder_names = {'RVS_G','RVS_B','RVS_DF','RVS_DA','RV_G','RV_B','RV_DF','RV_DA','RV_I'};
%folder_names = {'AV_DA_V','AV_DA_M','AV_DA_P','AV_DF_V','AV_DF_M','AV_DF_P'};
%folder_names = {'Explore','Balance','Exploit','Dynamic'};
%folder_names = {'A','B','C','D','E','F','G','H','I','J','K'};
%folder_names = {'M','N','O','P','B','C','D','E','G','I','J','K','L'};
%folder_names = {'kld2','kld3','kld4','kld5','kld6', 'kld7','O'};

%folder_names = {'A','B'};
% Obtener todas las subcarpetas válidas dentro del parent_folder
all_items = dir(parent_folder);
folder_names = {all_items([all_items.isdir] & ~startsWith({all_items.name}, '.')).name};

% Usar las carpetas encontradas como etiquetas de experimento
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

%% Guardar resultados combinados
output_csv = fullfile(parent_folder, 'combined_experiment_data.csv');
writetable(combinedTable, output_csv);
fprintf('Datos combinados guardados en: %s\n', output_csv);

%% BOXPLOTS + SUCCESS RATE
f1 = figure('Position',[100 100 3500 600]);  % más ancho
set(f1, 'Color', 'w');

f2 = figure('Position',[100 100 3500 600]);  % más ancho
set(f2, 'Color', 'w');

vars = {'error_total', 'iteration', 'exp_time', 'traj_length'};
titles = {'Total Error', 'Iterations', 'Time', 'Distance'};
ylabels = {'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
ylims = {[-3,10], [0,40], [0,300], [0,150]};

% vars = {'iteration', 'exp_time', 'traj_length'};
% titles = {'Iterations', 'Time', 'Distance'};
% ylabels = {'Iterations', 'Time (s)', 'Distance (m)'};
% ylims = {[0,40], [0,300], [0,150]};


% vars = {'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length'};
% titles = {'X error', 'Y error', 'Iterations', 'Time', 'Distance'};
% ylabels = {'Error (m)', 'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
% ylims = {[-1,10], [-3,4], [0,40], [0,300], [0,150]};

% vars = {'iteration', 'exp_time', 'traj_length'};
% titles = {'Iterations', 'Time', 'Distance'};
% ylabels = {'Iterations', 'Time (s)', 'Distance (m)'};
% ylims = {[0,40], [0,300], [0,150]};

n_metrics = 4;
n_subplots = length(vars);  % 4 métricas (success, efficiency, geom, info) + boxplots

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
bar(1:length(success_rates), success_rates * 100, 'FaceColor', green);

% Ajustes de ejes y etiquetas
ylim(ylim_success);
ylabel('Success rate (%)', 'FontWeight', 'bold');
xlabel('Experiment', 'FontWeight', 'bold');
t = title('Success', 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;

xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
set(gca, 'TickLabelInterpreter', 'none');  % <<--- Desactiva LaTeX
xtickangle(90);
grid on;
set(gca, 'FontSize', 11, 'FontWeight', 'bold');

%% Team efficiency

team_efficiency_vals = zeros(length(ordered_experiments), 1);
for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        team_efficiency_vals(i) = mean(exp_data.team_efficiency, 'omitnan');
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

bar(1:length(team_efficiency_vals), team_efficiency_vals, 'FaceColor', blue);
t = title('Efficiency ', 'FontWeight', 'bold');
ylabel('Team Efficiency (%)', 'FontWeight', 'bold');

t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;

xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
xtickangle(90);
ylim(ylim_eff);
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

bar(1:length(geom_scores), geom_scores*100, 'FaceColor', magenta);
ylim([0 100]);
ylabel('Geometric equity (%)', 'FontWeight', 'bold');
xlabel('Experiment', 'FontWeight', 'bold');
t = title('Geometry', 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
xtickangle(90);
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

bar(1:length(info_scores), info_scores*100, 'FaceColor', orange);
ylim([0 100]);
ylabel('Informative equity (%)', 'FontWeight', 'bold');
xlabel('Experiment', 'FontWeight', 'bold');
t = title('Informative', 'FontWeight', 'bold');
t.Units = 'normalized';
pos = t.Position;
pos(2) = pos(2) + 0.02;  % ajusta aquí
t.Position = pos;
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
xtickangle(90);
set(gca, 'FontSize', 11, 'FontWeight', 'bold');
grid on;


%% Subplots 2 a 6: boxplots
for k = 1:n_subplots
    figure(2)
    subplot(1, n_subplots, k);

    % === Fondo gris para las primeras 'divide' columnas ===
    hold on;
    num_experiments = numel(ordered_experiments);
    for j = 1:divide
        x0 = j - 0.5;
        x1 = j + 0.5;
        ylims_current = ylims{k};
        if isempty(ylims_current)
            ylims_current = ylim; % usa límites automáticos si no están definidos
        end
        patch([x0 x1 x1 x0], [ylims_current(1) ylims_current(1) ylims_current(2) ylims_current(2)], ...
              color_divide, 'EdgeColor', 'none');
    end

    % === Boxplot ===
    boxplot(combinedTable.(vars{k}), combinedTable.Experiment, ...
        'Colors', blue, 'Symbol', 'k+', 'Widths', 0.5, 'Whisker', 1.5);
    yline(0, '--g', 'LineWidth', 1.2);
    lines = findobj(gca, 'type', 'line', 'Tag', 'Median');
    set(lines, 'Color', red);
    set(findobj(gca,'type','line'),'LineWidth',1.2);
    grid on;
    xlabel('Experiment', 'FontWeight', 'bold');
    ylabel(ylabels{k}, 'FontWeight', 'bold');
    title(titles{k}, 'FontWeight', 'bold');
    if ~isempty(ylims{k}), ylim(ylims{k}); end
    set(gca, 'FontSize', 11, 'FontWeight', 'bold');
    xtickangle(90);
end


sgtitle('', 'FontSize', 16, 'FontWeight', 'bold');


%% Guardar figura
saveas(f1, fullfile(parent_folder, 'combined_metrics_boxplot.png'));
saveas(f2, fullfile(parent_folder, 'combined_results_boxplot.png'));
