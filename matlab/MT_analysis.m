% Cargar CSV
data = readtable('/home/mzarias//Workspace/MT_Marc/plume_model_learning/Results/MT_Simulation_Experiment/ABSOLUTE_VALUE/Done/combined_experiment_data.csv');

% Agrupar por experimento y calcular la media de cada métrica
metrics = varfun(@mean, data, ...
    'InputVariables', {'Success', 'exp_time', 'traj_length', 'error_total'}, ...
    'GroupingVariables', 'Experiment');

% Extraer valores
success = metrics.mean_Success;
exp_time = metrics.mean_exp_time;
traj_length = metrics.mean_traj_length;
error = metrics.mean_error_total;

% Normalizar: success (a maximizar)
success_norm = (success - min(success)) / (max(success) - min(success));

% Normalizar: exp_time, traj_length, error (a minimizar)
exp_time_norm = (max(exp_time) - exp_time) / (max(exp_time) - min(exp_time));
traj_length_norm = (max(traj_length) - traj_length) / (max(traj_length) - min(traj_length));
error_norm = (max(error) - error) / (max(error) - min(error));

% Ponderación (ajustable)
weights = [0.4, 0.4, 0.1, 0.1];

% Calcular puntuación final
scores = success_norm * weights(1) + ...
         exp_time_norm * weights(2) + ...
         traj_length_norm * weights(3) + ...
         error_norm * weights(4);

% Crear tabla de resultados
results = table(metrics.Experiment, scores, ...
    success, exp_time, traj_length, error, ...
    'VariableNames', {'Experiment', 'Score', 'Success', 'ExpTime', 'TrajLength', 'Error'});

% Ordenar de mayor a menor puntuación
results = sortrows(results, 'Score', 'descend');

% Mostrar top resultados
disp(results);

%%
% Cargar CSV
data = readtable('/home/mzarias//Workspace/MT_Marc/plume_model_learning/Results/MT_Simulation_Experiment/ABSOLUTE_VALUE/Done/combined_experiment_data.csv');

% Agrupar por experimento y calcular medias
metrics = varfun(@mean, data, ...
    'InputVariables', {'Success', 'exp_time', 'team_efficiency', ...
                       'informative_score', 'geometric_score'}, ...
    'GroupingVariables', 'Experiment');

% Extraer columnas
success = metrics.mean_Success;
exp_time = metrics.mean_exp_time;
team_eff = metrics.mean_team_efficiency;
info_eq = metrics.mean_informative_score;
geo_eq = metrics.mean_geometric_score;

% Normalizar (maximizar)
success_norm = (success - min(success)) / (max(success) - min(success));
team_eff_norm = (team_eff - min(team_eff)) / (max(team_eff) - min(team_eff));
info_eq_norm = (info_eq - min(info_eq)) / (max(info_eq - min(info_eq)));
geo_eq_norm = (geo_eq - min(geo_eq)) / (max(geo_eq - min(geo_eq)));

% Normalizar (minimizar)
exp_time_norm = (max(exp_time) - exp_time) / (max(exp_time) - min(exp_time));

% Definir pesos (puedes ajustarlos)
weights = [0.35, 0.25, 0.2, 0.1, 0.1];

% Calcular puntuación total
score = success_norm * weights(1) + ...
        exp_time_norm * weights(2) + ...
        team_eff_norm * weights(3) + ...
        info_eq_norm * weights(4) + ...
        geo_eq_norm * weights(5);

% Crear tabla de resultados
results = table(metrics.Experiment, score, ...
    success, exp_time, team_eff, info_eq, geo_eq, ...
    'VariableNames', {'Experiment', 'Score', 'Success', 'ExpTime', ...
                      'TeamEfficiency', 'InfoEquity', 'GeoEquity'});

% Ordenar resultados por puntuación descendente
results = sortrows(results, 'Score', 'descend');

% Mostrar resultados
disp(results);
