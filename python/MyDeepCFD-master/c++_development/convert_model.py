import torch
import torchvision
import scipy.io as sio
import numpy as np
# An instance of your model.
#model = torchvision.models.resnet18()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
f = open('/home/wjin/Workspace/continuous_STE/webots/data/map_nbr.txt', 'r')
m = int(f.read())
f.close()

DeepCFD = torch.load("Wanting_test.pt")

sx = 3.224360
sy = 2.772563
# when no source position is given
if(sx==-1 and sy==-1):
    adr = "/home/wjin/Workspace/continuous_STE/Results/test_maps/map{}/test_maps_x_for_MCMC.mat".format(m)
    mat_x = sio.loadmat(adr)
    array_x = mat_x.get('py_x')
    double_x = torch.from_numpy(array_x)
    x = double_x.float()
    x = x[:,:3,:,:]
    r = torch.from_numpy(mat_x.get('X')).float()
    c = torch.from_numpy(mat_x.get('Y')).float()
else:
    # make the input tensors using sx sy
    # mat_x = sio.loadmat('/data/FaezehRahbar/Projects/Data-driven_plume_model/Results/training_set_map1/dataset_mat/input_stationary_channels.mat')
    adr = "/home/wjin/Workspace/continuous_STE/Results/test_maps/map{}/test_map{}_flowdef_sdf_emptysourcepos.mat".format(m,m)
    mat_x = sio.loadmat(adr)
    array_x = mat_x.get('py_x')
    double_x = torch.from_numpy(array_x)
    x = double_x.float()
    # x = x[:,1:4,:,:]
    # print("making source dist map")
    r = torch.from_numpy(mat_x.get('X')).float()
    c = torch.from_numpy(mat_x.get('Y')).float()
    x[0,2,:,:] = 1/np.sqrt( (sx-r)**2 + (sy-c)**2 )

# An example input you would normally provide to your model's forward() method.
#example = torch.rand(1, 3, 224, 224)

# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
traced_script_module = torch.jit.trace(DeepCFD, x.to(device))

traced_script_module.save("DeepCFD_libtorch_model.pt")
