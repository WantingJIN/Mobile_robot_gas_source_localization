%% load data
clear all; close all; clc;

root_folder='/home/wjin/Workspace/continuous_STE/Khepera/Results_0818/continuous_clustering_ratio10/10/2023-08-18_16-54';

pc_folder=[root_folder '/pc'];
k4_folder=[root_folder '/k4'];
pdf = dlmread([pc_folder,'/posterior_pdf_3D.csv']);
goal_positions = dlmread([k4_folder, '/goal_position.csv']);
positions_his = dlmread([pc_folder, '/position_history.csv']);
measurements = dlmread([pc_folder, '/measurement_history.csv']);
cluster_bounding_boxes = dlmread([pc_folder, '/clustering_bounding_boxes.csv']);
params = my_text2struct([pc_folder, '/param_log.csv']);
anim_folder='/home/wjin/Workspace/continuous_STE/Results/ICRA2024/Final_campaign/Paper_plot/Experiment_result';
obst_file = ['/home/wjin/Workspace/continuous_STE/Khepera/Data/Obstacle_lists/' num2str(params.map_nbr) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end

entropy = dlmread([pc_folder, '/entropy.csv']);
real_source_position = [12.7, 1];

x=linspace(2.75,12.75,20);
y=linspace(0,4,20);
current_positions = [];
current_measurements=[];
k = 1;

%% plot for initial state
    f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
    ax1 = axes;
    pdf_initial = zeros(size(pdf(1,:),2),1);
    pdf_initial(:) = 1/size(pdf_initial,1);
    imagesc(ax1,y,x,reshape(pdf_initial,20,20))
%     ax1.XDir = 'reverse'; 
    ax1.YDir = 'reverse';
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
%     colormap winter; colorbar;
    hold on
    plot(positions_his(1,3),positions_his(1,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
 
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(0)]);
    ax1.TitleFontSizeMultiplier = 2;
    hold on
    ax2 = axes; 
    %scatter(ax2,current_positions(:,2),current_positions(:,1),30,current_measurements(:),'MarkerFaceColor','flat','MarkerEdgeColor','k');
    clim([0, max(measurements(:,2))]);
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    ax2.XDir = 'normal'; 
    ax2.YDir = 'reverse';
    ax2.Visible = 'off'; 
    ax2.XTick = []; 
    ax2.YTick = [];
    linkaxes([ax1, ax2]);
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; 
    colormap(ax2, parula) 
    colormap(ax1, mymap) 

    set([ax1,ax2],'Position', [.17 .11 .685 .815]); 
    cb1 = colorbar(ax1,'Position', [.20 .11 .0675 .815]); 
    %cb2 = colorbar(ax2,'Position', [.75 .11 .0675 .815]);
    %annotation(f,'textbox',[0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Measurement:', num2str(measurements(i,2))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
     annotation(f,'textbox',[0.45287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Entropy:', num2str(5.9915)},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',20);
    annotation(f,'textbox',[0.18287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'PDF'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',20);
    %annotation(f,'textbox',[0.75287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Measurement'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',20);

    hold off;
    xlim([0 4])
    ylim([2 13])
 
    saveas(f, [anim_folder,'/0.png']);

%% plot for each iteration
for i=1:size(pdf,1)
    f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
    ax1 = axes;
    imagesc(ax1,y,x,reshape(pdf(i,:),20,20))
%     ax1.XDir = 'reverse'; 
    ax1.YDir = 'reverse';
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
%     colormap winter; colorbar;
    hold on
    plot(goal_positions(i,3),goal_positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    for j = k:size (positions_his)
        if sqrt((positions_his(j,3)-goal_positions(i,3))^2+(positions_his(j,2)-goal_positions(i,2))^2) < 0.05
            break
        end
        current_positions = [current_positions;positions_his(j,2) positions_his(j,3)];
        current_measurements = [current_measurements; measurements(j,2)];
    end
   if i<size(pdf,1)
       if params.planning_strategy == 5
        cluster_num = cluster_bounding_boxes(i,1);
        if cluster_num > 0
            for cluster_idx = 1 : cluster_num
                x1 = cluster_bounding_boxes(i, 3 + (cluster_idx-1)*4);
                y1 = cluster_bounding_boxes(i, 4 + (cluster_idx-1)*4);
                x2 = cluster_bounding_boxes(i, 5 + (cluster_idx-1)*4 );
                y2 = cluster_bounding_boxes(i, 6 + (cluster_idx-1)*4);
                rectangle('Position',[y1 x1 (y2-y1) (x2-x1)], 'EdgeColor',"#80FF00",'LineWidth',4)
                hold on
            end
        end
      end
   end
   

    k = j;
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    ax1.TitleFontSizeMultiplier = 2;
    hold on
    ax2 = axes; 
    scatter(ax2,current_positions(:,2),current_positions(:,1),30,current_measurements(:),'MarkerFaceColor','flat','MarkerEdgeColor','k');
    clim([0, max(measurements(:,2))]);
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    ax2.XDir = 'normal'; 
    ax2.YDir = 'reverse';
    ax2.Visible = 'off'; 
    ax2.XTick = []; 
    ax2.YTick = [];
    linkaxes([ax1, ax2]);
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; 
    colormap(ax2, parula) 
    colormap(ax1, mymap) 

    set([ax1,ax2],'Position', [.17 .11 .685 .815]); 
    cb1 = colorbar(ax1,'Position', [.20 .11 .0675 .815]); 
    cb2 = colorbar(ax2,'Position', [.75 .11 .0675 .815]);
    %annotation(f,'textbox',[0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Measurement:', num2str(measurements(i,2))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
    annotation(f,'textbox',[0.45287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Entropy:', num2str(entropy(i,1))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',20);
    annotation(f,'textbox',[0.18287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'PDF'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',20);
    annotation(f,'textbox',[0.75287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Measurement'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',20);

    hold off;
    xlim([0 4])
    ylim([2 13])
 
    pause;
    drawnow
    saveas(f, [anim_folder,'/',num2str(i),'.png']);
end