% Leer datos
data = readtable('/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/MT_Simulation_Experiment/ABSOLUTE_VALUE/Done/combined_experiment_data.csv');

% Agrupar por experimento y calcular tasa de éxito promedio
grouped = groupsummary(data, "Experiment", "mean", "Success");

% Filtrar experimentos con tasa de éxito > 80%
filtered = grouped(grouped.mean_Success > 0.1, :);

% Crear gráfico de barras
figure;
bar(categorical(filtered.Experiment), filtered.mean_Success * 100); % convertir a %
ylabel('Success Rate (%)');
title('Experimentos con Success Rate > 80%');
xtickangle(90);
grid on;

%%
% Leer datos
data = readtable('/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/MT_Simulation_Experiment/ABSOLUTE_VALUE/Done/combined_experiment_data.csv');

% Calcular tasa de éxito promedio por experimento
grouped = groupsummary(data, "Experiment", "mean", "Success");

% Seleccionar experimentos con tasa de éxito > 80%
valid_experiments = grouped.Experiment(grouped.mean_Success > 0.1);

% Filtrar la tabla original con esos experimentos
filtered_data = data(ismember(data.Experiment, valid_experiments), :);

% Crear boxplot solo para experimentos filtrados
figure;
boxplot(filtered_data.exp_time, filtered_data.Experiment);

xlabel('Experimento');
ylabel('Simulation Time (s)');
title('Tiempos de Simulación (solo Experimentos con >80% Éxito)');
xtickangle(90);
grid on;
