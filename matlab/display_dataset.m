clear all; close all; clc;

% data = load('/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/datasets/data_base_grid.csv');
%data = load('/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/training_set_map1/training_map1_wind1.0_angle0/rr53/spx8.803374_spy3.453590.csv');
% results_folder = '../Results/test_maps/map8/';
results_folder = '../Results/training_random/13/';
[file,path] = uigetfile([results_folder '*.csv']);
data = load([path file]);

%% obstacles
map_nbr = maps_nbr_from_path(path);
% openfoam_folder = ['../openfoam/test_maps/map' map_nbr '/map' map_nbr '_wind1/'];
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
scatter(obstacle.X2(:), obstacle.Y2(:), 30, 'y', 'filled', 's');
hold on;
% source poistion
plot(source_position_x, source_position_y, 'dr','MarkerSize',10,'MarkerFaceColor','r');
% sensor positions
s = scatter(data(:,7), data(:,8), 10, 'k');
% concentrations
s = scatter(x, y, 10, c, 'filled');
axis equal;

%%
function map_nbr = maps_nbr_from_path(path)
B = regexp(path,'\d*','Match');
map_nbr = B{1};
end

