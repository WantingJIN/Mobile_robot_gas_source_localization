clear all; close all; clc;

general_folder='/home/durancea/plume_model_learning/Results/SR_MOX/10/1696237480617/101';
experiment_file_path = general_folder;


threeDpdf = dlmread([experiment_file_path,'/posterior_pdf_3D.csv']);
pdf = threeDpdf(1:2,:);
positions = dlmread([experiment_file_path, '/position_history.csv']);
anim_folder=[general_folder '/ani'];
params = my_text2struct([experiment_file_path,'/param_log.csv']);
map_nbr = params.map_nbr;
obst_file = ['/home/durancea/plume_model_learning/Khepera/Data/Obstacle_lists/' num2str(map_nbr) '_obstacles.txt'];
obst_fileID = fopen(obst_file);
C = textscan(obst_fileID,'%f %f %f %f', 'HeaderLines', 1);
for k=1:length(C{1,1})
            obstacles_x(:,k) = [C{1,1}(k) , C{1,3}(k), C{1,3}(k) , C{1,1}(k)];
            obstacles_y(:,k) = [C{1,2}(k) , C{1,2}(k), C{1,4}(k) , C{1,4}(k)];
end
measurements = dlmread([experiment_file_path, '/measurement_history.csv']); %measurement_history
entropy = dlmread([experiment_file_path, '/entropy.csv']);
real_source_position = [12.7 1];

x=linspace(2.75,12.75,20);
y=linspace(0,4,20);

% plot the initial pdf for video
f =figure('Position',[601.0000  111.4000  670.6000  1450.6000]);
imagesc(y,x,reshape(pdf(1,:),20,20))
hold on
plot(positions(1,3),positions(1,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
hold on
plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
hold on
patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
title(['Posterior pdf on X and Y \newline           iteration ', num2str(0)]);
xlabel('Y(m)'); ylabel('X(m)');
load MyColormap2; colormap(mymap); colorbar;caxis([0 Inf]);
annotation(f,'textbox',[0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Reading:', 'NaN'},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
annotation(f,'textbox',[0.60287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Entropy:', num2str(entropy(1,1))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
hold off;
xlim([0 4])
ylim([3 13])
ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
ax = gca; 
ax.TitleFontSizeMultiplier = 3; 
pause;
drawnow
saveas(f, [anim_folder,'/',num2str(0),'.png']);

test1 = size(pdf,1)

for i=2:size(pdf,1)
    test=i
    f =figure('Position',[601.0000  111.4000  670.6000  1450.6000]);
    imagesc(y,x,reshape(pdf(i,:),20,20))
    hold on
    plot(positions(i-1,3),positions(i-1,2),'ko','MarkerSize',25,'MarkerFaceColor','m');
    plot(real_source_position(2),real_source_position(1),'p','MarkerSize',40,'MarkerEdgeColor','blue','MarkerFaceColor',[51/255 153/255 1]);
    hold on
    patch(obstacles_y,obstacles_x ,[0 0 0], 'DisplayName','Obstacle','LineStyle', 'none');
    axis equal; axis([-0.1,4.1,2.4,12.6]);view(180,90);
    title(['Posterior pdf on X and Y \newline           iteration ', num2str(i-1)]);
    xlabel('Y(m)'); ylabel('X(m)');
    load MyColormap2; colormap(mymap); colorbar;caxis([0 Inf]);
    annotation(f,'textbox',[0.254287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Reading:', num2str(measurements(i-1,2))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
    annotation(f,'textbox',[0.60287457153379 0.025796129034299 0.104365076491284 0.0473684201637904],'String',{'Entropy:', num2str(entropy(i,1))},'LineStyle','none','HorizontalAlignment','center','FontWeight','bold','FontSize',24);
    hold off;
    xlim([0 4])
    ylim([3 13])
    ax = gca; ax.XDir = 'reverse'; ax.YDir = 'normal';
    ax = gca;
    ax.TitleFontSizeMultiplier = 3;
    pause;
    drawnow
    saveas(f, [anim_folder,'/',num2str(i-1),'.png']);
end
close all
