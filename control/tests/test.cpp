#include "DroneController.h"
int main(int argc, char const* argv[]){
  DroneController dronectrl(0.802, 0.1, 0.1, 0.1, 1280.0, 720.0);
  Eigen::Vector3d zyrPos={200,640,0};
  Eigen::Vector3d desVector={2,1,-1};
  Eigen::Vector3d ctrl=dronectrl.getVectorControl(1, zyrPos, 0, 0, desVector);
  std::cout<<ctrl<<"\n";
}
