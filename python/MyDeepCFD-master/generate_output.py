import torch
import pickle
import numpy as np
import scipy.io as sio
from functions import *
from torch.utils.data import TensorDataset

if __name__ == "__main__":
	# Loading dataset
    print(torch.__version__)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    #device = "cpu"
    # x = pickle.load(open("./dataX.pkl", "rb"))
    # y = pickle.load(open("./dataY.pkl", "rb"))
    # x = torch.FloatTensor(x)
    # y = torch.FloatTensor(y)
       
    mat_x = sio.loadmat('/home/wanting/DISAL/Research/KheperaOdor/data-driven_plume_model/Results/test_maps/test_maps_x_for_torch.mat')
    array_x = mat_x.get('py_x')
    double_x = torch.from_numpy(array_x)
    x = double_x.float()
    x=x[:,:3,:,:]
    # x=x[:,1:4:,:]
    mat_y = sio.loadmat('/home/wanting/DISAL/Research/KheperaOdor/data-driven_plume_model/Results/test_maps/test_maps_y_for_torch.mat')
    # mat_y = sio.loadmat('/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/training_random2/my_y_for_torch.mat')
    array_y = mat_y.get('py_y')
    double_y = torch.from_numpy(array_y)
    y = double_y.float()


    # All the data used as test data 
    train_data, test_data = split_tensors(x, y, ratio=0)
    
    train_dataset, test_dataset = TensorDataset(*train_data), TensorDataset(*test_data)        
    test_x, test_y = test_dataset[:]

    # load the saved model
    #DeepCFD = torch.load("myDeepCFD_model_Cavg_noWind_training_random2_1000_border_fixed_4layers_input180.pt")
    DeepCFD = torch.load("myDeepCFD_wanting_test")
    DeepCFD.eval()

    # visualize and save the output
    out = DeepCFD(test_x[:10].to(device))
    error = torch.abs(out.cpu() - test_y[:10].cpu())
    for s in range(test_x[:10].size(0)):
    	visualize(test_x[:10].cpu().detach().numpy(), test_y[:10].cpu().detach().numpy(), out.cpu().detach().numpy(), error.cpu().detach().numpy(), s)
    print("sum of all errors on c_avg: ", error[:,0,:,:].sum())