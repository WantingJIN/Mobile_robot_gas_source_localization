clear all; close all; clc;

% to plot all test maps in one figure

folder_path = '/home/durancea/plume_model_learning/openfoam/test_maps';
figure(1)
[ha, pos] = tight_subplot(4,3,[.03 .03],[.03 .03],[.05 .02]);
cmap = [ repmat([255,255,255]./255,0,1)      % < 0
         repmat([0 0 0]./255,33,1)           % 0 - 0.33
         repmat([255,255,255]./255,5001,1)   % > 50
];

map_nbr = [0 1 2 3 4 5 6 7 8 9 10];
for i=1:size(map_nbr,2)
    folder_name = ['/home/durancea/plume_model_learning/openfoam/test_maps/map' num2str(map_nbr(i)) '/map' num2str(map_nbr(i)) '_wind1/'];
    obstacles = load([folder_name num2str(map_nbr(i)) '_obstacle.mat']);
    p = 1:size(map_nbr,2);
    axes(ha(i));
    set(gcf,'position',[600,600,1000,250]);
    imagesc(obstacles.X2(1,:), obstacles.Y2(:,1), obstacles.obstacle_map);
    xlim([0.2,12.9])
    daspect([1.3 1 1])
    xticks(0:6:12)
    yticks(0:4:4)
    set(gca, 'YDir','normal')
    ax = gca;
    colormap(cmap)
end

saveas(gcf,[folder_path '/' 'physical_test_map_total' '.png']);

