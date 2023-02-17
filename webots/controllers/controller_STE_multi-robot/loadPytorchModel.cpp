#include "loadPytorchModel.h" // One-stop header.

void load_model_c() {
 

    // Initialize device
    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;

    // Load the number of maps
    int m;
    std::ifstream f("/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/webots/data/map_nbr.txt");
    f >> m;
    f.close();

    // Load the model
    torch::jit::script::Module DeepCFD = torch::jit::load("DeepCFD_libtorch_model.pt", device);

    // Initialize the positions
    float sx = 3.224360, sy = 2.772563;

    // Load mat file
    std::string mat_address = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps/map" + std::to_string(m) + "/test_map" + std::to_string(m) + "test_flowdef_sdf_emptysourcepos.mat";
    //convert string to char*
    std::cout << mat_address << std::endl;
    char *mat_adr = new char[mat_address.length() + 1];
    strcpy(mat_adr, mat_address.c_str());
    mat_t *mat;
    matvar_t *r = NULL, *c = NULL, *flow_def = NULL, *sdf = NULL, *reverse_sd= NULL;
    mat = Mat_Open(mat_adr, MAT_ACC_RDONLY);
    if (mat == NULL) {
        std::cout << "Error opening MAT file" << std::endl;
        //return 1;
    }
    //std::cout << mat-> << std::endl;
    //std::cout << mat<< std::endl;
    flow_def = Mat_VarRead(mat, "flow_def");
    sdf = Mat_VarRead(mat, "sdf");
    reverse_sd = Mat_VarRead(mat, "source_pos");
    r = Mat_VarRead(mat, "X");
    c = Mat_VarRead(mat, "Y");
    if (flow_def == NULL || sdf == NULL || reverse_sd == NULL ||r == NULL || c == NULL) {
        std::cout << "Error reading MAT file" << std::endl;
        //return 1;
    }
    // declare the tensor of dimension 1x3x64x64
    torch::Tensor x = torch::zeros({1, 3, 64, 64}, torch::TensorOptions().dtype(torch::kDouble));

    //convert to torch tensor
   
    torch:: Tensor flow_def_torch = torch::from_blob(flow_def->data, {(long long)flow_def->dims[0], (long long)flow_def->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
    torch:: Tensor sdf_torch = torch::from_blob(sdf->data, {(long long)sdf->dims[0], (long long)sdf->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
    torch:: Tensor reverse_sd_torch = torch::from_blob(reverse_sd->data, {(long long)reverse_sd->dims[0], (long long)reverse_sd->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
    
    x[0][0].slice(/*dim=*/0, /*start=*/0, /*end=*/flow_def_torch.size(0)).copy_(flow_def_torch);
    x[0][1].slice(/*dim=*/0, /*start=*/0, /*end=*/sdf_torch.size(0)).copy_(sdf_torch);
    x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/reverse_sd_torch.size(0)).copy_(reverse_sd_torch);

    torch::Tensor r_torch = torch::from_blob(r->data, {(long long)r->dims[0], (long long)r->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
    // Compare the original data and the tensor data
    //  for (int i = 0; i < r->dims[0]; i++) {
    //     for (int j = 0; j < r->dims[1]; j++) {
    //     std::cout << "Original data: " << ptr[i * r->dims[1] + j] << " Tensor value: " <<r_torch[i][j].item<double>() << std::endl;
    //   }
    // }
    torch::Tensor c_torch = torch::from_blob(c->data, {(long long)c->dims[0], (long long)c->dims[1]}, torch::TensorOptions().dtype(torch::kDouble));
    //std::cout << r_torch << std::endl;
    
    // Calculate the revert distance to the estimated gas source
    x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/r_torch.size(0)) = 1 / torch::sqrt( (sx - r_torch).pow(2) + (sy - c_torch).pow(2) );
    std::vector<torch::jit::IValue> inputs;
    //transpose the tensor
    torch::Tensor x_trans = x.permute({0, 1, 3, 2});
    // NOTE: The input tensor must be a float tensor
    inputs.push_back(x_trans.to(torch::kFloat).to(device));
    //std::cout << x_trans[0][1] << std::endl;
    // Execute the model and turn its output into a tensor.
    at::Tensor output = DeepCFD(inputs).toTensor();
    //std::cout << output << std::endl;

    std::vector<float> lpx = {9.0, 3.5, 3.0};
    std::vector<float> lpy = {2.4, 1, 2.7};
    std::vector<float> concentration;
    for (int i=0; i < lpx.size(); i++) {
        int idx_x = round((lpx[i] - min(r_torch[0]).item<float>())/(r_torch[1][0] - r_torch[0][0]).item<float>());
        int idx_y = round((lpy[i] - min(c_torch[0]).item<float>())/(c_torch[0][1] - c_torch[0][0]).item<float>());
        std::cout<< "idx_x is: "<<idx_x;
        std::cout<< " idx_y is: "<<idx_y << std::endl;
        std::vector<float> c_avg;
        c_avg.push_back(output[0][0][idx_y][idx_x].item<float>());
        std::cout << "The estimated concentration at (" << lpx[i] << ", " << lpy[i] << ") is " << output[0][0][idx_y][idx_x].item<float>() << std::endl;
    }
    
    Mat_VarFree(r);
    Mat_VarFree(c);
    Mat_Close(mat);
    //return 0;
}
