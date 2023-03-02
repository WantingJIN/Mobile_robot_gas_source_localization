experiment_folder = '/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/Experimental_Results/Results_0228/2023-02-28_19-47/';
experiment_measurement = dlmread([experiment_folder 'k4/measurements.csv']);
baseline_measurement = dlmread([experiment_folder 'k4/baseline_measurements.csv']);
webots_ground_truth = dlmread([experiment_folder 'Ground_truth_wb/measurements.csv']);
experiment_gas_con = experiment_measurement(:,3) - baseline_measurement(1,3);

%%%%%when reading scan file
%experiment_folder = '/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/Experimental_Results/Results_0228/log_K4_scan/'
%experiment_measurement = dlmread([experiment_folder 'measurements.csv']);
%experiment_gas_con = experiment_measurement(:,3)


for i = 1: size(experiment_gas_con)
    if experiment_gas_con(i,1) <0
        experiment_gas_con(i,1) =0;
    end
end
webots_gas_con = webots_ground_truth(:,3);
plot(experiment_gas_con,webots_gas_con,'o')
hold on
s = fitoptions('Method','NonlinearLeastSquares',...
               'Lower',[0,0],...
               'Upper',[Inf,max(cdate)],...
               'Startpoint',[1 1]);
f = fittype('a*(x-b)^n','problem','n','options',s);
[c2,gof2] = fit(experiment_gas_con,webots_gas_con,f,'problem',1)
plot(c2,'m')
%f = fit(experiment_gas_con, webots_gas_con)