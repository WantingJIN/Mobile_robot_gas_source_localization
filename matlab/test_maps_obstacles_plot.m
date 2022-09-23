clear all; close all; clc;

% to plot all test maps in one figure

folder_path = '/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/openfoam/test_maps';
figure(1)
[ha, pos] = tight_subplot(2,4,[.1 .03],[.1 .1],[.01 .01])
cmap = [ repmat([44,136,217],0,1)               % < 0
    repmat([241 197 45]./255,33,1)          % 0 - 0.33
%    repmat([255, 204, 0]./255,100,1)          % 0.33 - 1
%    repmat([153, 102, 255]./255,250,1)         % 1 - 2.5
%    repmat([102, 255, 51]./255,500,1)            % 2.5 - 5
%    repmat([255, 0, 255]./255,1100,1)         % 5 - 11
%    repmat([0 112 255]./255,2300,1)          % 11 - 23
%    repmat([255 255 0]./255,3500,1)          % 23 - 35
%    repmat([255 170 0]./255,5000,1)          % 35 - 50
   repmat([44,136,217]./255,5001,1)            % > 50
];
for i=1:8
    folder_name = ['/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/openfoam/test_maps/map' num2str(i) '/map' num2str(i) '_wind1/'];
    obstacles = load([folder_name num2str(i) '_obstacle.mat']);
    p = [1, 2, 3, 4, 5, 6, 7 ,8];
    %subplot(2,4,p(i));
    axes(ha(i));
    set(gcf,'position',[600,600,1000,250]);
    imagesc(obstacles.X2(1,:), obstacles.Y2(:,1), obstacles.obstacle_map);
    xlim([0.2,12.9])
    daspect([1.3 1 1])
    set(gca, 'YDir','normal')
    ax = gca;
    colormap(cmap)
    %title([num2str(p(i))]);
    %ax.TitleFontSizeMultiplier =2
end
%set(ha(1:8),'XTickLabel',''); set(ha(1:8),'YTickLabel','')
saveas(gcf,[folder_path '/' 'test_map_total' '.png']);
%%
for i=1:8
    folder_name = ['/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/openfoam/test_maps/map' num2str(i) '/map' num2str(i) '_wind1/'];
    obstacles = load([folder_name num2str(i) '_obstacle.mat']);
    figure(2)
    set(gcf,'position',[600,600,1000,250]);
    imagesc(obstacles.X2(1,:), obstacles.Y2(:,1), obstacles.obstacle_map);
    daspect([1 1 1])
    set(gca, 'YDir','normal')
    colormap('bone')
    title(['Test map ' num2str(p(i))]);
    save_path = ['/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/openfoam/test_maps/map' num2str(i)];
    saveas(gcf,[save_path '/' num2str(i) '.png']);
    
end

