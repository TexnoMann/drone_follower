#include "ardrone_control/DroneController.h"
int main(int argc, char const* argv[]){
  DroneController dronectrl(0.802, 0.1, 0.1, 0.1, 1280.0, 720.0);
  Eigen::Vector4d xyzrPos={0,0,-360,0};
  Eigen::Vector3d desVector={2,1,-1};
  std::vector<float> ctrl=dronectrl.getVectorControl(1, xyzrPos, 0, 0, desVector);

}
