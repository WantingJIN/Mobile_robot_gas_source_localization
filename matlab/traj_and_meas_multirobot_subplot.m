close all; clc; clear all;

% choose the last folder in log files
folder_list = dir('/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/test/9');
folder_list = folder_list(~ismember({folder_list.name},{'.','..','datasets'}));
exp_number = length(folder_list);

for i = 1:exp_number
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    if folder_list(i).isdir == 0
        continue;
    end
    
    % multiple robots
    robot_list = dir(folder_name);
    robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
    robot_list = robot_list([robot_list.isdir]);
    ID = arrayfun(@(struct) str2num(struct.name), robot_list);
    params = my_text2struct([[folder_name, '/', num2str(ID(1))], '/param_log.csv']);
    gas_feature = params.gas_feature;
    
    % create subplot figure
    num_robots = params.number_of_robots;
    f2all = figure('Name', 'Trajectory & Measurements');
    set(f2all, 'Position', get(0, 'Screensize'));

    % adjust subplot positions
    height = 0.9 / num_robots;
    gap = 0.02; % gap between plots
    pos = arrayfun(@(k) [0.2, 1 - (k * height + gap), 0.7, height - gap], 1:num_robots, 'UniformOutput', false);

    for j = 1:num_robots
        folder_name_ID = [folder_name, '/', num2str(ID(j))];
        position_history = dlmread([folder_name_ID, '/position_history.csv']);
        measurement_history = dlmread([folder_name_ID, '/measurement_history.csv']);
        positions = dlmread([folder_name_ID, '/goal_position.csv']);
        positions(:,4) = 0;
        trajectory = dlmread([folder_name_ID, '/trajectory.csv']);
        trajectory(:,4) = 0;
        trajectory = [positions(1,:); trajectory];
        measurements = dlmread([folder_name_ID, '/measurements.csv']);
        expected_a_posteriori = dlmread([folder_name_ID, '/expected_a_posteriori.csv']);
        source_real_position = params.source_position;
        
        % subplot single column
        ax = subplot(num_robots,1,j);
        set(ax, 'Position', pos{j});
        hold on;
        set(gca, 'Color', 'w');
        grid on; axis equal;

        xlim([3 13]); 
        ylim([0 4]); 
        zlim([0 2]);

        if gas_feature == 0 || gas_feature == 2
            scatter3(position_history(:,2), position_history(:,3), position_history(:,4), 25, measurement_history(:,2), 'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k');
            scatter3(positions(:,2), positions(:,3), positions(:,4), 25, measurements(:,3), 'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k', 'LineWidth', 0.05);
            colorbar;
        elseif gas_feature == 1
            scatter3(position_history(:,2), position_history(:,3), position_history(:,4), 10, measurement_history(:,3), 'MarkerFaceColor', 'flat', 'MarkerEdgeColor', 'k');
        end

        plot3(trajectory(:,2), trajectory(:,3), trajectory(:,4), 'LineWidth', 1);
        plot3(source_real_position(1), source_real_position(2), source_real_position(3), 'g+', 'MarkerSize', 20, 'LineWidth', 3);
        plot3(expected_a_posteriori(end,2), expected_a_posteriori(end,3), 0, 'rx', 'MarkerSize', 20, 'LineWidth', 3);
        plot3(positions(1,2), positions(1,3), positions(1,4), 'o', 'MarkerSize', 8, 'LineWidth', 2);
        
        xlabel('X (m)'); ylabel('Y (m)'); view(180,90);

        % obstacles
        obstacles_x = [];
        obstacles_y = [];
        obst_file = ['../openfoam/test_maps/map' num2str(params.map_nbr) '/map' num2str(params.map_nbr) '_wind0.75/' num2str(params.map_nbr) '_obstacles.txt'];
        obst_fileID = fopen(obst_file);
        C = textscan(obst_fileID, '%f %f %f %f', 'HeaderLines', 1);
        fclose(obst_fileID);

        for k = 1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k), C{1,3}(k), C{1,3}(k), C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k), C{1,2}(k), C{1,4}(k), C{1,4}(k)];
        end
        patch(obstacles_x, obstacles_y, [0.75 0.75 0.75], 'DisplayName', 'Obstacle', 'LineStyle', 'none');

        hold off;

        % title to the left side of the subplot
        annotation('textbox', [0.05, pos{j}(2), 0.1, height], 'String', ['Robot ', num2str(ID(j))], ...
            'EdgeColor', 'none', 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');
    end

    % save figure
    saveas(f2all, [folder_name, '/trajectory.png']);
end
