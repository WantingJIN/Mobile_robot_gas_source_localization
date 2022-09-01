clear all; close all; clc;

% Check the test map environments
general_folder = '/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps';
map_number = 8;
file_name = [general_folder, '/map', num2str(map_number),'/test_map', num2str(map_number), '.environment'];

fileID = fopen(file_name,'r');

env_vertex = textscan(fileID,'%f %f','CommentStyle','//')

env_x = env_vertex{1};
env_y = env_vertex{2};

plot(env_x, env_y)
axis equal