#include <torch/script.h> // One-stop header.
#include <torch/torch.h>
#include <numpy/ndarrayobject.h>
#include <fstream>
#include <string>

#include <iostream>
#include <memory>

//#include <matio.h>
//#include <Eigen/Dense>


int main(int argc, const char* argv[]) {
 

  // Create a vector of inputs.
  // std::vector<torch::jit::IValue> inputs;
  // inputs.push_back(torch::ones({1, 3, 224, 224}));

  // // Execute the model and turn its output into a tensor.
  // at::Tensor output = module.forward(inputs).toTensor();
  // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/5) << '\n';
   // ...

 
  // mat_t *matfp;
  // matvar_t *matvar;

  // matfp = Mat_Open(mat_adr, MAT_ACC_RDONLY);
  // matvar = Mat_VarRead(matfp, "py_x");

  // Eigen::MatrixXd array_x(matvar->dims[0], matvar->dims[1]);
  // memcpy(array_x.data(), matvar->data, matvar->nbytes);
  // Mat_VarFree(matvar);
  // Mat_Close(matfp);

  // Eigen::Tensor<double, 2> x(array_x.rows(), array_x.cols());
  // x.array() = array_x.array();

// Perform calculations on tensor x


    // Initialize device
    torch::Device device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;

    // Load the number of maps
    int m;
    std::ifstream f("/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/webots/data/map_nbr.txt");
    f >> m;
    f.close();

    // Load the model
    torch::jit::script::Module DeepCFD = torch::jit::load("Wanting_test.pt", device);

    // Initialize the positions
    float sx = 3.224360, sy = 2.772563;

    // Load the .mat file
    std::string adr = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps/map" + std::to_string(m) + "/test_map" + std::to_string(m) + "_flowdef_sdf_emptysourcepos.mat";
    PyObject* mat_x = PyImport_ImportModule("scipy.io");
    PyObject* loadmat_func = PyObject_GetAttrString(mat_x, "loadmat");
    PyObject* mat_x_dict = PyObject_CallFunction(loadmat_func, const_cast<char*>((adr + std::string("'")).c_str()));
    
    // Convert array_x to a numpy array
    PyObject* array_x_numpy = PyDict_GetItemString(mat_x_dict, "py_x");

    // Convert the numpy array to a torch tensor
    torch::Tensor double_x = torch::from_numpy(array_x_numpy);

    torch::Tensor x = double_x.to(torch::kFloat32);

    // Convert X and Y to torch tensors
    torch::Tensor r = torch::from_numpy(PyDict_GetItemString(mat_x_dict, "X")).to(torch::kFloat32);
    torch::Tensor c = torch::from_numpy(PyDict_GetItemString(mat_x_dict, "Y")).to(torch::kFloat32);

    // Perform the operation
    x[0][2].slice(/*dim=*/0, /*start=*/0, /*end=*/r.size(0)) = 1 / torch::sqrt( (sx - r).pow(2) + (sy - c).pow(2) );

    // Release the numpy array
    Py_DECREF(mat_x_dict);
    Py_DECREF(loadmat_func);
    Py_DECREF(mat_x);
    return 0;

}


// #include <torch/torch.h>
// #include <iostream>

// int main() {
//     // Check if CUDA is available
//     if (torch::cuda::is_available()) {
//         std::cout << "CUDA is available! Using CUDA device" << std::endl;
//         torch::Device device(torch::kCUDA);
//     }
//     else {
//         std::cout << "CUDA is not available! Using CPU device" &#8203;`oaicite:{"index":0,"invalid_reason":"Malformed citation << std::endl;\n        torch::Device device(torch::kCPU);\n    }\n\n    // Read map number from file\n    std::ifstream map_file(\"/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/webots/data/map_nbr.txt\");\n    int m;\n    map_file >>"}`&#8203; m;
//     map_file.close();
//     std::cout << "Map number: " << m << std::endl;

//     // Load pre-trained model
//     torch::jit::script::Module DeepCFD = torch::jit::load("Wanting_test.pt");

//     // Load input data
//     std::string adr = "/home/wjin/Workspace/plume_model_learning_wind_speed_0.75/Results/test_maps/map" + std::to_string(m) + "/test_map" + std::to_string(m) + "_flowdef_sdf_emptysourcepos.mat";
//     torch::Tensor x = torch::from_file(adr);
//     x = x.to(torch::kFloat32);
//     // x = x[{{},{1,4},{},{}}];
//     torch::Tensor r = torch::from_file(adr, "X");
//     r = r.to(torch::kFloat32);
//     torch::Tensor c = torch::from_file(adr, "Y");
//     c = c.to(torch::kFloat32);
//     x[{{0},{2},{},{}}] = 1 / torch::sqrt( (sx - r).pow(2) + (sy - c).pow(
