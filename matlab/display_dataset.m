clear all; close all; clc;

%results_folder = '../Results/test_maps/map1/';
results_folder = '../Results/training_random/181/';
[file,path] = uigetfile([results_folder '*.csv']);
data = load([path file]);

%% obstacles
map_nbr = maps_nbr_from_path(path);
%openfoam_folder = ['../openfoam/test_maps/map' map_nbr '/map' map_nbr '_wind1/'];
openfoam_folder = ['../openfoam/training_random/training_random_wind1.0_angle0/' map_nbr '/'];
obstacle = load([openfoam_folder maps_nbr_from_path(path) '_obstacle.mat']);
obstacle.obstacle_map = (obstacle.obstacle_map == 0);
obstacle.obstacle_map = obstacle.obstacle_map(:);
obstacle.X2 = obstacle.X2(:);
obstacle.Y2 = obstacle.Y2(:);

x = data(:,7);
y = data(:,8);
c = data(:,10);

x(c==0) = [];
y(c==0) = [];
c(c==0) = [];
obstacle.X2(obstacle.obstacle_map == 0) = [];
obstacle.Y2(obstacle.obstacle_map == 0) = [];
obstacle.obstacle_map(obstacle.obstacle_map == 0) = [];

%% source position
C = regexp(file,'\d+\.?\d*','match');
source_position_x = str2double(C{1}); 
source_position_y = str2double(C{2});

%% display
% obstacles
% define the color map
cmap = [ repmat([1 1 1],0,1)               % < 0
   repmat([114, 123, 175]./255,33,1)          % 0 - 0.33
   repmat([229, 189, 199]./255,100,1)          % 0.33 - 1
   repmat([196, 171, 204]./255,250,1)         % 1 - 2.5
   repmat([234, 195, 179]./255,500,1)            % 2.5 - 5
   repmat([243, 185, 146]./255,1100,1)         % 5 - 11
   repmat([0 112 255]./255,2300,1)          % 11 - 23
   repmat([255 255 0]./255,3500,1)          % 23 - 35
   repmat([255 170 0]./255,5000,1)          % 35 - 50
   repmat([255 0 0]./255,5001,1)            % > 50
];
scatter(obstacle.X2(:), obstacle.Y2(:), 30, 'y', 'filled', 's');
hold on;
% source poistion
plot(source_position_x, source_position_y, 'dr','MarkerSize',10,'MarkerFaceColor','r');
% sensor positions
s = scatter(data(:,7), data(:,8), 10, 'k');
% concentrations
s = scatter(x, y, 10, c, 'filled');
colorbar 
colormap(cmap)
axis equal;

%%
function map_nbr = maps_nbr_from_path(path)
B = regexp(path,'\d*','Match');
map_nbr = B{1};
end

