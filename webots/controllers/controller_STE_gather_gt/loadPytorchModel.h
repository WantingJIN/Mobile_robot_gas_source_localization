#include <torch/script.h> // One-stop header.
#include <torch/torch.h>
//#include <numpy/ndarrayobject.h>
#include <fstream>
#include <string>

#include <iostream>
#include <memory>
//#include <opencv2/opencv.hpp>
#include <matio.h>
#include <cmath>
//#include <eigen3/Eigen/Dense>

// NOTE: Download the matio library from https://sourceforge.net/projects/matio/files/matio/1.5.17/matio-1.5.17.tar.gz/download
// and compile it following the ReadME.txt file in the downloaded folder.
// The FindMATIO.cmake file is required for the CMAKE to find the matio library.
// Downloaded it from https://github.com/ami-iit/matio-cpp/blob/master/cmake/FindMATIO.cmake


void load_model_c();
