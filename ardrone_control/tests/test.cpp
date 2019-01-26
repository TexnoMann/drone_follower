#include "ardrone_control/DroneController.h"
int main(int argc, char const* argv[]){
  DroneController dronectrl(0.802, 0.1, 0.1, 0.1, 1280.0, 720.0);
  std::vector<double> xyzrPos={0,0,-360,0};
  std::vector<double> desVector={2,1,-1,0};
  std::vector<float> ctrl=dronectrl.getVectorControl(1, xyzrPos, 0, 0, desVector);

}
