clear all; close all; clc;

% to plot all test maps in one figure

folder_path = '/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/openfoam/test_maps';
figure(1)
for i=1:8
    folder_name = ['/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/openfoam/test_maps/map' num2str(i) '/map' num2str(i) '_wind1/'];
    obstacles = load([folder_name num2str(i) '_obstacle.mat']);
    p = [1, 2, 3, 4, 5, 6, 7 ,8];
    subplot(2,4,p(i));
    set(gcf,'position',[600,600,1000,250]);
    imagesc(obstacles.X2(1,:), obstacles.Y2(:,1), obstacles.obstacle_map);
    xlim([0,12.9])
    daspect([1 1 1])
    set(gca, 'YDir','normal')
    title(['Test map ' num2str(p(i))]);
end
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