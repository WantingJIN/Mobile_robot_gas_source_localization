#include <torch/script.h> // One-stop header.
#include <torch/torch.h>
//#include <numpy/ndarrayobject.h>
#include <fstream>
#include <string>

#include <iostream>
#include <memory>
//#include <opencv2/opencv.hpp>
#include <matio.h>
//#include <Eigen/Dense>

// NOTE: I downloaded the matio library from https://sourceforge.net/projects/matio/files/matio/1.5.17/matio-1.5.17.tar.gz/download
// and compiled it following the ReadME.txt file in the downloaded folder.
// The FindMATIO.cmake file is required for the CMAKE to find the matio library.
// I downloaded it from https://github.com/ami-iit/matio-cpp/blob/master/cmake/FindMATIO.cmake


int main(int argc, const char* argv[]) {
 

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
    std::string mat_address = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps/map" + std::to_string(m) + "/test_map" + std::to_string(m) + "_flowdef_sdf_emptysourcepos.mat";
    //convert string to char*
    std::cout << mat_address << std::endl;
    char *mat_adr = new char[mat_address.length() + 1];
    strcpy(mat_adr, mat_address.c_str());
    mat_t *mat;
    matvar_t *py_x = NULL, *r = NULL, *c = NULL;
    mat = Mat_Open(mat_adr, MAT_ACC_RDONLY);
    if (mat == NULL) {
        std::cout << "Error opening MAT file" << std::endl;
        return 1;
    }
    //std::cout << mat-> << std::endl;
    //std::cout << mat<< std::endl;
    py_x = Mat_VarRead(mat, "py_x");
    r = Mat_VarRead(mat, "X");
    c = Mat_VarRead(mat, "Y");
    if (py_x == NULL || r == NULL || c == NULL) {
        std::cout << "Error reading MAT file" << std::endl;
        return 1;
    }
     


    //convert to torch tensor
    torch::Tensor x = torch::from_blob(py_x->data, {(long long)py_x->dims[0], (long long)py_x->dims[1], (long long)py_x->dims[2], (long long)py_x->dims[3]}, torch::TensorOptions().dtype(torch::kDouble));
    //double * ptr = (double *) r->data;
    //  std::cout<< ptr << std::endl;
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
    // NOTE: The input tensor must be a float tensor
    inputs.push_back(x.to(torch::kFloat).to(device));
    //std::cout << DeepCFD(inputs) << std::endl;
    // Execute the model and turn its output into a tensor.
    at::Tensor output = DeepCFD(inputs).toTensor();
    //std::cout << output << std::endl;
  
    Mat_VarFree(py_x);
    Mat_VarFree(r);
    Mat_VarFree(c);
    Mat_Close(mat);
    return 0;

}
