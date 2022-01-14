function [CC, U] = openfoam_steadyState_CC_U(case_path)
%case_path = '/data/FaezehRahbar/Projects/Data-driven_plume_model/openfoam/map1/map1_wind1';
folder_list = dir(case_path);
folder_list = folder_list([folder_list.isdir]);
folder_list = folder_list(~ismember({folder_list.name},{'.','..','constant','system','VTK'}));

%U file
fileID = fopen([case_path,'/',folder_list(end).name,'/U']);
U = cell2mat(textscan(fileID,'(%f %f %f)', 'HeaderLines', 22));
fclose(fileID);

%cc files
fileID = fopen([case_path,'/',folder_list(1).name,'/Cx']);
ccx = cell2mat(textscan(fileID,'%f', 'HeaderLines', 22));
fclose(fileID);
fileID = fopen([case_path,'/',folder_list(1).name,'/Cy']);
ccy = cell2mat(textscan(fileID,'%f', 'HeaderLines', 22));
fclose(fileID);
fileID = fopen([case_path,'/',folder_list(1).name,'/Cz']);
ccz = cell2mat(textscan(fileID,'%f', 'HeaderLines', 22));
fclose(fileID);

CC = [ccx, ccy, ccz];
end 