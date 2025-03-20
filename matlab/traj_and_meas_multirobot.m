%  Plots and stores the robot trajectory, entropy and concentration
%  measurements for all experiments in folder_list

close all; clc; clear all;

% choose the last folder in log files
folder_list = dir('/home/mzarias/Workspace/MT_Marc/plume_model_learning/Results/test/9');
folder_list = folder_list(~ismember({folder_list.name},{'.','..','datasets'}));
exp_number = length(folder_list);

colors = [1, 0.7, 0.4; 0.5, 0.9, 0.5; 0.4, 0.6, 1]; % Define a colormap for different robots

%%
for i=1:exp_number
    folder_name = [folder_list(i).folder, '/', folder_list(i).name];
    if(folder_list(i).isdir==0) continue; end
    % multiple robots
    robot_list = dir(folder_name);
    robot_list = robot_list(~ismember({robot_list.name},{'.','..'}));
    robot_list = robot_list([robot_list.isdir]);
    ID = arrayfun(@(struct) str2num(struct.name),robot_list);
    params = my_text2struct([[folder_name, '/', num2str(ID(1))], '/param_log.csv']);
    gas_feature = params.gas_feature;

    f2all = figure('Name','measurements','Position',[686 656 794 418]); 
    hold on;
    
    for j=1:params.number_of_robots
        % data :
        folder_name_ID = [folder_name, '/', num2str(ID(j))];
        position_history = dlmread([folder_name_ID, '/position_history.csv']); %position_history
        measurement_history = dlmread([folder_name_ID, '/measurement_history.csv']); %measurement_history
        positions = dlmread([folder_name_ID, '/goal_position.csv']); %position_history
        positions(:,4) = 0;
        trajectory = dlmread([folder_name_ID, '/trajectory.csv']);
        trajectory(:,4) = 0;
        trajectory = [positions(1,:); trajectory];
        measurements = dlmread([folder_name_ID, '/measurements.csv']); %measurement_history
        entropy = dlmread([folder_name_ID, '/entropy.csv']);
        expected_a_posteriori = dlmread([folder_name_ID, '/expected_a_posteriori.csv']);
        max_a_posteriori = dlmread([folder_name_ID, '/max_a_posteriori.csv']);
        if(isnan(expected_a_posteriori(end,2))), expected_a_posteriori(end,2) = max_a_posteriori(end,2); end
        if(isnan(expected_a_posteriori(end,3))), expected_a_posteriori(end,3) = max_a_posteriori(end,3); end
        
        % ground truth and other params
        params = my_text2struct([folder_name_ID, '/param_log.csv']);
        source_real_position = params.source_position;
        
        % obstacles
        obstacles_x=[];
        obstacles_y=[];
        obst_file = ['../openfoam/test_maps/map' num2str(params.map_nbr) '/map' num2str(params.map_nbr) '_wind0.75/' num2str(params.map_nbr) '_obstacles.txt'];
        obst_fileID = fopen(obst_file);
        C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
        for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
        end
        
        % Define color for this robot
        robot_color = colors(j, :);
        
        % Plot trajectory and measurements
        figure(f2all);
        scatter3(position_history(:,2),position_history(:,3),position_history(:,4),10,robot_color,'filled','MarkerEdgeColor','k');
        plot3(trajectory(:,2),trajectory(:,3),trajectory(:,4),'Color',robot_color,'LineWidth',1);
        
        plot3(source_real_position(1),source_real_position(2),source_real_position(3),'k+','MarkerSize',20,'LineWidth',3,'DisplayName','Source real position'); %real source position
        plot3(expected_a_posteriori(end,2), expected_a_posteriori(end,3), 0, 'x', 'Color', 'k', 'MarkerSize', 16, 'LineWidth', 5);
        plot3(expected_a_posteriori(end,2),expected_a_posteriori(end,3),0, 'x','Color', robot_color, 'MarkerSize',14,'LineWidth',3,'DisplayName','Final expected value for source position'); %where the robot thinks the source is
        plot3(positions(1,2), positions(1,3), positions(1,4), 'o', 'MarkerSize', 10, 'LineWidth', 2, 'LineStyle', 'none', 'Color', 'k');
        plot3(positions(1,2), positions(1,3), positions(1,4), 'o', 'MarkerSize', 6, 'LineWidth', 2, 'LineStyle', 'none', 'Color', 'k');
        plot3(positions(1,2),positions(1,3), positions(1,4),'DisplayName','Start point','MarkerSize',8,'Marker','o','LineWidth',2,'LineStyle','none', 'Color', robot_color);
        patch(obstacles_x,obstacles_y ,[0.75 0.75 0.75],'DisplayName','Obstacle', 'LineStyle', 'none');
    end
    
    hold off;
    grid on; axis equal; axis([2.25 13.25 0 4 0 2]);
    xlabel('X (m)');ylabel('Y (m)');
    view(180,90);
    title('Trajectory and measurements');
    a = get(gca,'XTickLabel'); set(gca,'XTickLabel',a,'FontName','Times','fontweight','bold','fontsize',14);
    
    % save
    saveas(f2all,[folder_name '/trajectory.png']);
end
