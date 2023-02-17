#include "gasMap2D.h"


GasMap2D::GasMap2D(double sigma, double gamma, double scaling_param, double wind_speed, double rco){
  avg_WRt=0;
  counter_WRt=0;

  configureKernelAlgorithm(sigma, gamma, scaling_param, wind_speed, rco);
  initialize_gas_map_features();
}

void GasMap2D::initialize_gas_map_features(){
  x_min = map_x_min;
  x_max = map_x_max;
  y_min = map_y_min;
  y_max = map_y_max;
  x_resol = (x_max - x_min) / map_x_nbins;
  y_resol = (y_max - y_min) / map_y_nbins;
  x_update_range = kp.rco / x_resol;
  y_update_range = kp.rco / y_resol;
}

void GasMap2D::configureKernelAlgorithm(double sigma, double gamma, double scaling_param, double wind_speed, double rco){
  kp.sigma = sigma;
  kp.gamma = gamma;
  kp.scaling_param = scaling_param;
  kp.wind_speed = wind_speed;
  kp.rco = rco;
  gasAverConMap = Eigen::Matrix<double,map_x_nbins,map_y_nbins>::Zero();
  gasWeightMap = Eigen::Matrix<double,map_x_nbins,map_y_nbins>::Zero();
  gasConfidenceMap = Eigen::Matrix<double,map_x_nbins,map_y_nbins>::Zero();
  gasWeightConMap = Eigen::Matrix<double,map_x_nbins,map_y_nbins>::Zero();
  
  setGaussianDivisor();
}

void GasMap2D::runKernelAlgorithm(Position current_position, float current_odor){

  double x_dist = 0, y_dist = 0; //distance from sampled point from center of cells
  //normalize received odor sample
  int close_coord[3]= {0};
  int idx_x = round((current_position.x - x_min) / x_resol);
  int idx_y = round((current_position.y - y_min) / y_resol);
  int idx_x_start = (idx_x - x_update_range > 0) ? (idx_x - x_update_range) : 0;
  int idx_x_end = (idx_x + x_update_range < map_x_nbins) ? (idx_x + x_update_range) : map_x_nbins;
  int idx_y_start = (idx_y - y_update_range > 0) ? (idx_y - y_update_range) : 0;
  int idx_y_end = (idx_y + y_update_range < map_y_nbins) ? (idx_y + y_update_range) : map_y_nbins;
  //printf("x_pos is %f, y_pos is %f, odor_con is %f \n", current_position.x, current_position.y, current_odor);
  //printf("idx x start is %d, idx x end is %d, idx y start is %d, idx y end is %d \n", idx_x_start, idx_x_end, idx_y_start, idx_y_end);
  for(int i = idx_x_start; i < idx_x_end; i++){
    for(int j = idx_y_start; j < idx_y_end; j++){
      x_dist = i * x_resol + x_min - current_position.x;
      y_dist = j * y_resol + y_min - current_position.y;
      float wcells = compute_gaussian(x_dist, y_dist);
      //printf("gasWeightMap(%d,%d) is %f ", i, j, gasWeightMap(i,j));
      gasWeightMap(i,j) += wcells;
      //printf(" Upodated gasWeightMap(%d,%d) is %f ", i, j, gasWeightMap(i,j));
      gasWeightConMap(i,j) += current_odor * wcells;
      //printf(" Upodated gasWeightConMap(%d,%d) is %f ", i, j, gasWeightConMap(i,j));
      gasConfidenceMap(i,j) = 1 - exp(-gasWeightMap(i,j) / pow(kp.scaling_param,2));
      gasAverConMap(i,j) = gasWeightConMap(i,j) / gasWeightMap(i,j);
      //printf(" Upodated gasAverConMap(%d,%d) is %f \n", i, j, gasAverConMap(i,j));
    }
  }
}

//////////////////////////////////////////////////// PRIVATE FUNCTIONS ///////////////////////////////////////

// void GasMap2D::setSigmaMatrix(){
//   //if (kp.wind_speed ==0){
//     _sigma_matrix << kp.sigma,0,
//                 0,kp.sigma;
//   //}else{
//     // FIXME: check if this is correct
//     // float bc= kp.sigma/sqrt(1+kp.gamma*kp.wind_speed/kp.sigma);
//     // _sigma_matrix << kp.sigma+kp.gamma*kp.wind_speed,0,0,
//     //              0,bc,0,
//     //              0,0,bc;
//   //}
// }

// void GasMap2D::setGaussianDivisor(){
//   _gaussian_divisor= 1/(pow(2*M_PI,3/2)*pow(_sigma_matrix.determinant(),0.5));
// }
void GasMap2D::setGaussianDivisor(){
   _gaussian_divisor= 1/(2*M_PI*pow(kp.sigma,2));
}

float GasMap2D::compute_gaussian(const double x_dist, const double y_dist){
    float num= (pow(x_dist,2)+ pow(y_dist,2))/ (2 * pow(kp.sigma,2));
    return exp(-num)/_gaussian_divisor;
}

void GasMap2D::retrieve_readings_from_gasmap(const vector<Position> history_position, vector<double> &concentrations_from_map){
  for(int i=0; i<history_position.size(); i++){
    int idx_x = round((history_position[i].x - x_min) / x_resol);
    int idx_y = round((history_position[i].y - y_min) / y_resol);
    concentrations_from_map.push_back(gasAverConMap(idx_x,idx_y));
  }
}