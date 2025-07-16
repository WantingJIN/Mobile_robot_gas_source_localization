clear; clc; close all;

%% Parameters
divide = 0;
color_divide = [0.85, 0.85, 0.85];

%% Configuración
% parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week18/Exp4/';
% folder_names = {'No','2','4','6','8','10','12','14','16'};
parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week21/B/';
%folder_names = {'AVS_G', 'AVS_B', 'RVS_G', 'RVS_B','AV_G','AV_B', 'RV_G', 'RV_B'};
%folder_names = {'AVS', 'RVS', 'AV', 'RV'};
%folder_names = {'AVS_G','AVS_B','AVS_DF','AVS_DA','AV_G','AV_B','AV_DF','AV_DA','AV_I'};
%folder_names = {'4KAV_I','8KAV_I','10KAV_I','12KAV_I','14KAV_I','16KAV_I','18KAV_I','NOKAV_I'};
%folder_names = {'RVS_G','RVS_B','RVS_DF','RVS_DA','RV_G','RV_B','RV_DF','RV_DA','RV_I'};
%folder_names = {'AV_DA_V','AV_DA_M','AV_DA_P','AV_DF_V','AV_DF_M','AV_DF_P'};
%folder_names = {'Explore','Balance','Exploit','Dynamic'};
%folder_names = {'A','B','C','D','E','F','G','H','I','J','K'};
%folder_names = {'M','N','O','P','B','C','D','E','G','I','J','K','L'};
folder_names = {'RV_IC_V','RV_IC_VLD','RV_IC_VLP','RV_IC_RG'};
combinedTable = table();

%% Colores y nombres
blue = [44,136,217]/255;
red = [217,83,79]/255;
green = [91,183,91]/255;

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
    combinedTable = [combinedTable; T];
end

%% Guardar resultados combinados
output_csv = fullfile(parent_folder, 'combined_experiment_data.csv');
writetable(combinedTable, output_csv);
fprintf('Datos combinados guardados en: %s\n', output_csv);

%% BOXPLOTS + SUCCESS RATE
f = figure('Position',[100 100 2100 600]);

vars = {'x_error', 'y_error', 'iteration', 'exp_time', 'traj_length'};
titles = {'X error', 'Y error', 'Iterations', 'Time', 'Distance'};
ylabels = {'Error (m)', 'Error (m)', 'Iterations', 'Time (s)', 'Distance (m)'};
ylims = {[-1,10], [-3,4], [0,40], [0,300], [0,150]};

% vars = {'iteration', 'exp_time', 'traj_length'};
% titles = {'Iterations', 'Time', 'Distance'};
% ylabels = {'Iterations', 'Time (s)', 'Distance (m)'};
% ylims = {[0,40], [0,300], [0,150]};


%% Subplot 1: Success rate por experimento
subplot(1,6,1);
hold on;

% Experimentos ordenados
ordered_experiments =   {'RV_IC_V','RV_IC_VLD','RV_IC_VLP','RV_IC_RG'};
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
title('Success rate', 'FontWeight', 'bold');
xticks(1:length(ordered_experiments));
xticklabels(ordered_experiments);
set(gca, 'TickLabelInterpreter', 'none');  % <<--- Desactiva LaTeX
xtickangle(90);
grid on;
set(gca, 'FontSize', 11, 'FontWeight', 'bold');


%% Subplots 2 a 6: boxplots
for k = 1:5
    subplot(1,6,k+1); % shift by 1

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
saveas(f, fullfile(parent_folder, 'combined_results_boxplot.png'));
