#include "ardrone_control/DroneController.h"


DroneController::DroneController(double limAngleCamY,double limAngleCamZ, double Kx, double Ky, double Kz, double YSizeCam, double ZSizeCam){
  this->_limAngleCamZ=limAngleCamZ*M_PI/180;
  this->_limAngleCamY=limAngleCamY*M_PI/180;
  this->_Kx=Kx;
  this->_Ky=Ky;
  this->_Kz=Kz;
  this->_alpha0 = limAngleCamY*M_PI/(180*YSizeCam);
  this->_ball_radius=0.032;
  //Camera size in pixel
  this->_YSizeCam=YSizeCam;
  this->_ZSizeCam=ZSizeCam;
  this->_maxControlVector={1,1,1};
  //matrix for P controller
  this->_controlMatrix<< Kx, 0, 0,
                          0, Ky, 0,
                          0, 0, Kz;
  this->_xOffset=0.21;
}

std::vector<float> DroneController::getVectorControl(std::vector<double> xyzrObjectV, double alphaDrone, double phiDrone, std::vector<double> desiredVector){

  Eigen::Vector4d xyzrObject={xyzrObjectV[0],xyzrObjectV[1],xyzrObjectV[2],xyzrObjectV[3]};
  Eigen::Vector3d desV={desiredVector[0],desiredVector[1],desiredVector[2]};

  std::vector<float> controlv(4);
  Eigen::Vector3d control=_controlMatrix*(_getCurrentVector(xyzrObject,alphaDrone,phiDrone)-desV);
  //Chech overcontrol
  for(int i=0; i<3;i++){
    if(abs(control[i])>_maxControlVector[i]){
      control[i]=copysign(_maxControlVector[i],control[i]);
    }
    controlv[i]=(float)control[i];
  }
  controlv[3]=0.0;

  #ifdef DEBUG
  // std::cout<<controlv[0]<<"\n";
  #endif
  return controlv;
}


//Get vector to object of base frame on drone
Eigen::Vector3d DroneController::_getCurrentVector(Eigen::Vector4d xyzrObject, double alphaDrone, double phiDrone){
  Eigen::Vector3d currentVector;

  double l =0.9827* _ball_radius/(sin(_alpha0*xyzrObject[3]/2)) - 0.1754;
  currentVector[0]=l*cos(_alpha0*xyzrObject[2])*cos(_alpha0*xyzrObject[1])+cos(alphaDrone)*0.21;
  currentVector[1]=l*sin(_alpha0*xyzrObject[1])*cos(_alpha0*xyzrObject[2]);
  currentVector[2]=l*sin(_alpha0*xyzrObject[2])+sin(alphaDrone)*0.21;

  std::cout << l << "\n";

  #ifdef DEBUG
  std::cout<<"Vector for Object: \n"<<currentVector<<"\n";
  #endif
  currentVector=_getTransformRotMatrix(phiDrone, alphaDrone)*currentVector;

  return currentVector;
}

//Get rotation matrix for frame trnsformation
Eigen::Matrix3d DroneController::_getTransformRotMatrix(double alpha, double phi){
  Eigen::Matrix3d RMatrix;
  RMatrix = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(phi,  Eigen::Vector3d::UnitY());
  return RMatrix;
}
