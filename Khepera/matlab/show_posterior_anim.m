clear all; close all; clc;

root_folder='/home/durancea/plume_model_learning/Results/Experimental_Results/Results_0727/2023-07-27_16-01';

pc_folder=[root_folder '/pc'];
k4_folder=[root_folder '/k4'];
pdf = dlmread([pc_folder,'/posterior_pdf_3D.csv']);
positions = dlmread([k4_folder, '/goal_position.csv']);
params = my_text2struct([pc_folder, '/param_log.csv']);
%anim_folder='/home//Workspace/continuous_STE/Khepera/DataKhepera/Results/ani'
obst_file = ['/home/durancea/plume_model_learning/Khepera/Data/Obstacle_lists/' num2str(params.map_nbr) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end
measurements = dlmread([pc_folder, '/measurement_history.csv']); %measurement_history
entropy = dlmread([pc_folder, '/entropy.csv']);
real_source_position = [12.7, 2];

x=linspace(2.75,12.75,20);
y=linspace(0,4,20);
f =figure('Position',[601.0000  111.4000  697.6000  1450.6000]);
for i=1:size(pdf,1)
    imagesc(y,x,reshape(pdf(i,:),20,20))
    hold on
    plot(positions(i,3),positions(i,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar;
    annotation(f,'textbox',[0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Measurement:', num2str(measurements(i,2))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
    annotation(f,'textbox',[0.60287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Entropy:', num2str(entropy(i,1))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
    hold off;
    xlim([0 4])
    ylim([2 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
    ax = gca;
    ax.TitleFontSizeMultiplier = 3;
    pause;
    drawnow
    %saveas(f, [anim_folder,'/',num2str(i),'.png']);
end