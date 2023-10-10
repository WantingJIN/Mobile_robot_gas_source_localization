clear all; close all; clc;

% plot selected trajectories for the thesis
general_folder = '../Results/results_test_maps';
folder_list = dir([general_folder '/**']);
folder_list = folder_list(~ismember({folder_list.name},{'.','..', '101', '102','103'}));
folder_list = folder_list(~ismember({folder_list.folder}, general_folder));
folder_list = folder_list([folder_list.isdir]);
number = length(folder_list);

% selected trajectories
% sel_traj = [...
%     0, 1614305194731
%     1, 1614308550673
%     2, 1614514078898
%     3, 1614356191737
%     4, 1614616179589
%     5, 1614414169262
%     6, 1614591944736
%     7, 1614422570165
%     8, 1614605746471];

sel_traj = [1, 1661960855092];

% the figure
f2all = figure('Name','measurements','Position',[-1561 656 794 415]); 
addpath("functions");
for i=1:length(sel_traj)
    %% get data
    folder_name = [general_folder, '/', num2str(sel_traj(i,1)), '/' num2str(sel_traj(i,2)), '/101'];
    positions = dlmread([folder_name, '/goal_position.csv']); 
    positions(:,4) = 0;
    trajectory = dlmread([folder_name, '/trajectory.csv']);
    trajectory(:,4) = 0;
    trajectory = [positions(1,:); trajectory];
    measurements = dlmread([folder_name, '/measurements.csv']); 
    expected_a_posteriori = dlmread([folder_name, '/expected_a_posteriori.csv']);
    max_a_posteriori = dlmread([folder_name, '/max_a_posteriori.csv']);
    if(isnan(expected_a_posteriori(end,2))), expected_a_posteriori(end,2) = max_a_posteriori(end,2); end
    if(isnan(expected_a_posteriori(end,3))), expected_a_posteriori(end,3) = max_a_posteriori(end,3); end
    params = my_text2struct([folder_name, '/param_log.csv']);
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
    
    %% plot
    figure;
%     subplot(5,2, sel_traj(i,1)+1)
%     axes0 = axes('Parent',f2all,'Position',[0.113627204030227 0.245631966946655 0.775 0.815]);
    hold on;
    scatter3(positions(:,2),positions(:,3),positions(:,4),10,measurements(:,3),'DisplayName','Measurements','MarkerFaceColor','flat','MarkerEdgeColor','k', 'LineWidth', 0.01);
    plot3(trajectory(:,2),trajectory(:,3),trajectory(:,4),'DisplayName','Trajectory','LineWidth',1);%lines
    plot3(source_real_position(1),source_real_position(2),source_real_position(3),'g+','MarkerSize',10,'LineWidth',2,'DisplayName','Source real position'); %real source position
    plot3(expected_a_posteriori(end,2),expected_a_posteriori(end,3),0, 'rx','MarkerSize',10,'LineWidth',2,'DisplayName','Final expected value for source position'); %where the robot thinks the source is
    plot3(positions(1,2),positions(1,3), positions(1,4),'DisplayName','Start point','MarkerSize',8,'Marker','o','LineWidth',2,'LineStyle','none');
    %     plot3(position_history(:,2),position_history(:,3),position_history(:,4),'k');
    patch(obstacles_x,obstacles_y ,[0.6 0.8 1],'DisplayName','Obstacle', 'LineStyle', 'none');
    hold off;
%     set(axes0,'Ytick',0:1:4)
    grid on; axis equal; axis([2.25 13.25 0 4 0 2]);
%     c = colorbar('southoutside'); c.set('Position',[0.136020151133501 0.115853490011817 0.285894206549118 0.0545725751510899]); load MyColormap.mat; colormap(mymap); %colormap jet;
    xlabel('X (m)');ylabel('Y (m)');
    view(180,90);
%     legend1 = legend(axes0,'show');
%     set(legend1,'Position',[0.469773299748109 0.0272789076115389 0.413937271835497 0.221351976214818]);
%     annotation(f2all,'textbox',[0.234287457153379 0.185796129034299 0.104365076491284 0.0473684201637904],'String',{'Concentration'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold');
%     title(['Map ' num2str(sel_traj(i,1))]);
    a = get(gca,'XTickLabel'); set(gca,'XTickLabel',a,'FontName','Times','fontweight','bold','fontsize',9);
    hold off;
    
    %% save
    pic_name = [general_folder, '/', num2str(sel_traj(i,1)), '.png'];
    exportgraphics(gca, pic_name); 
end