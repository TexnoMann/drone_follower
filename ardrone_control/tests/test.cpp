#include "ardrone_control/DroneController.h"
int main(int argc, char const* argv[]){
  DroneController dronectrl(92,51.75, 0.1, 0.1, 0.1, 1280.0, 720.0);
  std::vector<double> xyzrPos={0,100,0,30};
  std::vector<double> desVector={1,0,0};
  std::vector<float> ctrl=dronectrl.getVectorControl(xyzrPos, 0, 0, desVector);

}
