%  Plots and stores the robot trajectory, entropy and concentration
%  measurements for all experiments in folder_list

close all; clc; clear all;

% choose the last folder in log files
folder_list = dir('/home/wjin/Workspace/continuous_STE/Results/info_planning/MOX/map0/belief_clustering');
folder_list = folder_list(~ismember({folder_list.name},{'.','..','datasets'}));
exp_number = length(folder_list);


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

    f1all = figure('Position',[-1561   656   570   418]);
    f2all = figure('Name','measurements','Position',[686 656 794 418]); 
    f0 = figure('Name','Expected a posteriori evolution','Position', [-1599, 1, 300, 486]);
    
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
%         posterior_over_time = dlmread([folder_name_ID,  '/posterior.csv']);
%         posterior = posterior_over_time(end-7999:end)';
%         posterior = posterior./sum(posterior);
        % ground truth and other params
        params = my_text2struct([folder_name_ID, '/param_log.csv']);
        source_real_position = params.source_position;
        Q_min = params.min_maxs(1); Q_max = params.min_maxs(2);
        source_position_min = [params.min_maxs(3),params.min_maxs(5)];
        source_position_max = [params.min_maxs(4),params.min_maxs(6)];
        a_y_min = params.min_maxs(7); a_y_max = params.min_maxs(8);
        b_y_min = params.min_maxs(9); b_y_max = params.min_maxs(10);
        Q_nbins = params.nbins(1);
        x_nbins = params.nbins(2);
        y_nbins = params.nbins(3);
        a_y_nbins = params.nbins(4);
        b_y_nbins = params.nbins(5);
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
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Mesurement points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        % overlaped for all robots
        figure(f1all);
        subplot(2,1,1);
        hold on;
        plot(measurement_history(:,2));
        title('concentration measurement over time');
        hold off;
        subplot(2,1,2);
        hold on;
        histogram(measurement_history(:,2));
        title('measurement histogram');
        hold off;

        
        figure(f2all);
        axes0 = axes('Parent',f2all,'Position',[0.113627204030227 0.245631966946655 0.775 0.815]);
        hold on;
        if gas_feature == 0 || gas_feature == 2
             scatter3(position_history(:,2),position_history(:,3),position_history(:,4),25,measurement_history(:,2),'DisplayName',['Measurements ID ', num2str(ID(j))],'MarkerFaceColor','flat','MarkerEdgeColor','k');hold on;
             scatter3(positions(:,2),positions(:,3),positions(:,4),25,measurements(:,3),'DisplayName',['Measurements ID ', num2str(ID(j))],'MarkerFaceColor','flat','MarkerEdgeColor','k', 'LineWidth', 0.05);hold on;
             c = colorbar('southoutside'); c.set('Position',[0.136020151133501 0.115853490011817 0.285894206549118 0.0545725751510899]);  load MyColormap.mat; colormap(mymap); %colormap jet;
        end
        if gas_feature == 1
            scatter3(position_history(:,2),position_history(:,3),position_history(:,4),10,measurement_history(:,3),'DisplayName',['Measurements ID ', num2str(ID(j))],'MarkerFaceColor','flat','MarkerEdgeColor','k');hold on;
        end

        plot3(trajectory(:,2),trajectory(:,3),trajectory(:,4),'DisplayName','Trajectory','LineWidth',1);%lines
        plot3(source_real_position(1),source_real_position(2),source_real_position(3),'g+','MarkerSize',20,'LineWidth',3,'DisplayName','Source real position'); %real source position
        plot3(expected_a_posteriori(end,2),expected_a_posteriori(end,3),0, 'rx','MarkerSize',20,'LineWidth',3,'DisplayName','Final expected value for source position'); %where the robot thinks the source is
        plot3(positions(1,2),positions(1,3), positions(1,4),'DisplayName','Start point','MarkerSize',8,'Marker','o','LineWidth',2,'LineStyle','none');
        patch(obstacles_x,obstacles_y ,[0.6 0.8 1],'DisplayName','Obstacle', 'LineStyle', 'none');
        hold off;
        set(axes0,'Ytick',0:1:4)
        grid on; axis equal; axis([2.25 13.25 0 4 0 2]);
        xlabel('X (m)');ylabel('Y (m)');
        view(180,90);
        legend1 = legend(axes0,'show');
        set(legend1,'Position',[0.469773299748109 0.0272789076115389 0.413937271835497 0.221351976214818]);
        annotation(f2all,'textbox',[0.234287457153379 0.185796129034299 0.104365076491284 0.0473684201637904],'String',{'Concentration'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold');
        title('Trajectory and measurements');
        a = get(gca,'XTickLabel'); set(gca,'XTickLabel',a,'FontName','Times','fontweight','bold');
        hold off;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Expected a posteriori %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(f0);
        
        % Subplot 1: X-coordinate estimation
        subplot(3,1,1);
        plot(expected_a_posteriori(:,2), '*-', 'DisplayName', 'Expected');  % Expected value for X-coordinate
        hold on;
        axis([-inf inf source_position_min(1) source_position_max(1)]);
        plot(max_a_posteriori(:,2), 'd-b', 'DisplayName', 'Max a posteriori');  % Maximum a posteriori estimate for X-coordinate
        plot([0 size(max_a_posteriori,1)], [source_real_position(1), source_real_position(1)], 'g', 'DisplayName', 'Ground Truth');  % Ground truth X-coordinate
        grid on;
        title('X');
        %legend('show');  % Display the legend

        % Subplot 2: Y-coordinate estimation
        subplot(3,1,2);
        plot(expected_a_posteriori(:,3), '*-', 'DisplayName', 'Expected');  % Expected value for Y-coordinate
        hold on;
        plot(max_a_posteriori(:,3), 'd-b', 'DisplayName', 'Max a posteriori');  % Maximum a posteriori estimate for Y-coordinate
        plot([0 size(max_a_posteriori,1)], [source_real_position(2), source_real_position(2)], 'g', 'DisplayName', 'Ground Truth');  % Ground truth Y-coordinate
        grid on;
        axis([-inf inf source_position_min(2) source_position_max(2)]);
        title('Y');
        legend('show');  % Display the legend
        
        % Subplot 3: X-coordinate estimation
        subplot(3,1,3);
        hold on;
        plot(entropy(:,1),'DisplayName',['ID ', num2str(ID(j))]);
        title('entropy');
        
        % subplot(3,2,2);
        % plot(expected_a_posteriori(:,4),'*-');
        % hold on;
        % plot(max_a_posteriori(:,4),'d-b')
        % grid on; axis([-inf inf Q_min Q_max]);
        % title('Q');
        % 
        % subplot(3,2,4);
        % plot(expected_a_posteriori(:,5),'*-');
        % hold on;
        % plot(max_a_posteriori(:,5),'d-b')
        % grid on; axis([-inf inf a_y_min a_y_max]);
        % title('sigma_a');
        % 
        % subplot(3,2,6);
        % plot(expected_a_posteriori(:,6),'*-');
        % hold on;
        % plot(max_a_posteriori(:,6),'d-b')
        % grid on; axis([-inf inf b_y_min b_y_max]);
        % title('sigma_b');
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Marginals %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         f1 = figure('Name','Posterior PDF','Position',[-799, 1, 800, 1086]);
% 
%         posterior_pdf = reshape(posterior,x_nbins, y_nbins, Q_nbins); %a_y_nbins, b_y_nbins
%         
%         % marginal_x = sum(sum(posterior_pdf,1),2);
%         marginal_x = sum(posterior_pdf,[2,3]);
%         marginal_x = marginal_x(:);
%         subplot(3,2,1);
%         range = source_position_min(1):(source_position_max(1)-source_position_min(1))/x_nbins:source_position_max(1);
%         bar((range(1:end-1)+range(2:end))/2, marginal_x);
%         hold on;
%         plot([expected_a_posteriori(end,2), expected_a_posteriori(end,2)], [0,1],'r-.');
%         plot([source_real_position(1), source_real_position(1)], [0,1],'g');
%         title('X');
%         
%         % marginal_y = sum(sum(posterior_pdf,1),3);
%         marginal_y = sum(posterior_pdf,[1,3]);
%         marginal_y = marginal_y(:);
%         subplot(3,2,3);
%         range = source_position_min(2):(source_position_max(2)-source_position_min(2))/y_nbins:source_position_max(2);
%         bar((range(1:end-1)+range(2:end))/2, marginal_y);
%         hold on;
%         plot([expected_a_posteriori(end,3), expected_a_posteriori(end,3)], [0,1],'r-.');
%         plot([source_real_position(2), source_real_position(2)], [0,1],'g');
%         title('Y');
%         
%         marginal_q = sum(posterior_pdf,[1,2]);
%         marginal_q = marginal_q(:);
%         subplot(3,2,2);
%         range = Q_min:(Q_max-Q_min)/Q_nbins:Q_max;
%         bar((range(1:end-1)+range(2:end))/2, marginal_q);
%         hold on;
%         plot([expected_a_posteriori(end,4), expected_a_posteriori(end,4)], [0,1],'r-.');
%         title('Q');
%         
% %         marginal_a = sum(posterior_pdf,[1,2,3,5]);
% %         marginal_a = marginal_a(:);
% %         subplot(3,2,4);
% %         range = a_y_min:(a_y_max-a_y_min)/a_y_nbins:a_y_max;
% %         bar((range(1:end-1)+range(2:end))/2, marginal_a);
% %         hold on;
% %         plot([expected_a_posteriori(end,5), expected_a_posteriori(end,5)], [0,1],'r-.');
% %         title('sigma_a');
% %         
% %         marginal_b = sum(posterior_pdf,[1,2,3,4]);
% %         marginal_b = marginal_b(:);
% %         subplot(3,2,6);
% %         range = b_y_min:(b_y_max-b_y_min)/b_y_nbins:b_y_max;
% %         bar((range(1:end-1)+range(2:end))/2, marginal_b);
% %         hold on;
% %         plot([expected_a_posteriori(end,6), expected_a_posteriori(end,6)], [0,1],'r-.');
% %         title('sigma_b');
%         
%         % save
%         saveas(f1,[folder_name_ID '/posterior_marginal.png']);
%         close(f1);
%         
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %% Posterior_3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         
%         f0 = figure('Position', [-1506 613 570 418]);
%         f1 = figure('Position', [-1533 76 570 418]);
%         
%         % calculation grid
%         X = [0:0.5:19.9]+0.05;
%         Y = [0:0.2:3.9]+0.05;
%         %Z = [0:0.2:1.9]+0.05;
%         %[X_grid,Y_grid,Z_grid] = meshgrid(Y,X,Z);% %Y,Z,X
%         
%         % plot
%         figure(f0);
%         plot(posterior);
% 
%         % calculation grid (WT)
%         %X = linspace(2.5,12.5,40)+0.05;
%         %Y = [0:0.2:3.9]+0.05;
%         [X_grid,Y_grid] = meshgrid(Y,X);
%         
%         % 3D display
%         posterior_6D = reshape(posterior,x_nbins, y_nbins, Q_nbins);
%         posterior_3D = sum(posterior_6D, 3);
%         figure(f1);
%         scatter(X_grid(:),Y_grid(:),2, posterior_3D(:));
%         load MyColormap; colormap(mymap); colorbar;
%         axis equal;
%         
%         % expected value
%         sum(X_grid(:) .* posterior_3D(:));
%         sum(Y_grid(:) .* posterior_3D(:));
%         
%         % save
%         saveas(f0,[folder_name_ID '/posterior_3D_raw.png']);
%         saveas(f1,[folder_name_ID '/posterior_3D.png']);
%         close(f0);close(f1);
        
    end
    % save
    saveas(f1all,[folder_name '/measurements_entropy.png']);
    saveas(f2all,[folder_name '/trajectory.png']);
    saveas(f0,[folder_name '/estimation_evolution.png']);    
    
    %close(f1all); 
    %lose(f2all);
    %close(f0);
end