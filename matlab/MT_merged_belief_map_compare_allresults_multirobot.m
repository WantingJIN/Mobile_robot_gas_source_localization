clear; clc; close all;

%% Configuración
% parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week18/Exp4/';
% folder_names = {'No','2','4','6','8','10','12','14','16'};
parent_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/Week18/Exp6/';
%folder_names = {'AVS_G', 'AVS_B', 'RVS_G', 'RVS_B','AV_G','AV_B', 'RV_G', 'RV_B'};
%folder_names = {'AVS', 'RVS', 'AV', 'RV'};
folder_names = {'AV_G_V', 'AV_B_V', 'AV_G_VL', 'AV_B_VL'};
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
% ordered_experiments = {'No','2','4','6','8','10','12','14','16'};
%ordered_experiments = {'AV','RV'};
%ordered_experiments = {'AVS_G', 'AVS_B', 'RVS_G', 'RVS_B','AV_G','AV_B', 'RV_G', 'RV_B'};
%ordered_experiments = {'AVS', 'RVS', 'AV', 'RV'}
ordered_experiments = {'AV_G_V', 'AV_B_V', 'AV_G_VL', 'AV_B_VL'};
success_rates = zeros(length(ordered_experiments), 1);

for i = 1:length(ordered_experiments)
    exp_data = combinedTable(strcmp(combinedTable.Experiment, ordered_experiments{i}), :);
    if ~isempty(exp_data)
        success_rates(i) = sum(exp_data.Success) / height(exp_data);
    else
        success_rates(i) = NaN;
    end
end

bar(categorical(ordered_experiments, ordered_experiments, 'Ordinal', true), success_rates*100, 'FaceColor', green);
ylim([0, 100]);
ylabel('Success rate (%)', 'FontWeight', 'bold');
xlabel('Experiment', 'FontWeight', 'bold');
title('Success rate', 'FontWeight', 'bold');
grid on;
set(gca, 'FontSize', 11, 'FontWeight', 'bold');
xtickangle(90); % Girar etiquetas del eje X a 90 grados

%% Subplots 2 a 6: boxplots
for k = 1:5
    subplot(1,6,k+1); % shift by 1
    boxplot(combinedTable.(vars{k}), combinedTable.Experiment, ...
        'Colors', blue, 'Symbol', 'k+', 'Widths', 0.5, 'Whisker', 1.5);
    hold on;
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
    xtickangle(90); % Girar etiquetas del eje X a 90 grados
end

sgtitle('', 'FontSize', 16, 'FontWeight', 'bold');


%% Guardar figura
saveas(f, fullfile(parent_folder, 'combined_results_boxplot.png'));
