% Ruta base
exp_folder = '/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/cluster_analysis/';

% Obtener lista de carpetas dentro de Exp7 (ej. Run1, Run2...)
run_folders = dir(exp_folder);
run_folders = run_folders([run_folders.isdir]);  % solo carpetas
run_folders = run_folders(~ismember({run_folders.name}, {'.','..'})); % eliminar . y ..

% IDs de los robots
robots = {'101', '102', '103'};

% Colores por robot (fijo)
colors = lines(length(robots)); 

for i = 1:length(run_folders)
    run_name = run_folders(i).name;
    run_path = fullfile(exp_folder, run_name);

    % Crear figura nueva para esta corrida
    figure('Color', 'w'); hold on;

    for j = 1:length(robots)
        robot_id = robots{j};
        data_file = fullfile(run_path, robot_id, 'measurements.csv');

        if exist(data_file, 'file')
            data = readtable(data_file);

            % Extraer datos
            concentration = data{:,3};
            time = data{:,end};

            % Eliminar ceros o negativos para escala log
            valid_idx = concentration > 0;
            concentration = concentration(valid_idx);
            time = time(valid_idx);

            % Graficar
            plot(time, concentration, ...
                'Color', colors(j,:), ...
                'DisplayName', ['Robot ', robot_id], ...
                'LineWidth', 1.2);
        else
            warning('No se encontró: %s', data_file);
        end
    end

    % Configurar gráfico para esta corrida
    xlabel('Time (seconds)');
    ylabel('Concentration');
    title(['Relative value evolution: ', run_name]);
    set(gca, 'YScale', 'log');
    legend('show');
    grid on;

    % (Opcional) Guardar figura automáticamente
    % saveas(gcf, fullfile(run_path, ['plot_', run_name, '.png']));
end
