function ground_truth_struct = my_text2struct(file_name)

param_file = fopen(file_name);
params = textscan(param_file,'%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
%params = textscan(param_file,'%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
ground_truth_struct = struct;

for i=1:size(params{1},1)
    array = [];
    for j=2:size(params,2)-1
        if(~isnan(params{j}(i)))
            array = [array, params{j}(i)];
        end
    end
    ground_truth_struct.(params{1}{i}) = array;
end

