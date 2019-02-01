#include "ardrone_control/DroneController.h"


DroneController::DroneController(double limAngleCamY,double limAngleCamZ, double Kx, double Ky, double Kz, double YSizeCam, double ZSizeCam){
  this->_limAngleCamZ=limAngleCamZ;
  this->_limAngleCamY=limAngleCamY;
  this->_Kx=Kx;
  this->_Ky=Ky;
  this->_Kz=Kz;
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

std::vector<float> DroneController::getVectorControl(double heighDrone, std::vector<double> xyzrObjectV, double alphaDrone, double phiDrone, std::vector<double> desiredVector){

  Eigen::Vector4d xyzrObject={xyzrObjectV[0],xyzrObjectV[1],xyzrObjectV[2],xyzrObjectV[3]};
  Eigen::Vector3d desV={desiredVector[0],desiredVector[1],desiredVector[2]};

  std::vector<float> controlv(4);
  Eigen::Vector3d control=_controlMatrix*(desV-_getCurrentVector(heighDrone,xyzrObject,alphaDrone,phiDrone));
  //Chech overcontrol
  for(int i=0; i<3;i++){
    if(abs(control[i])>_maxControlVector[i]){
      control[i]=copysign(_maxControlVector[i],control[i]);
    }
    controlv[i]=(float)control[i];
  }
  controlv[3]=0.0;

  #ifdef DEBUG
  std::cout<<controlv[0]<<"\n";
  #endif
  return controlv;
}


//Get vector to object of base frame on drone
Eigen::Vector3d DroneController::_getCurrentVector(double heighDrone, Eigen::Vector4d xyzrObject, double alphaDrone, double phiDrone){
  Eigen::Vector3d currentVector;
  // double angle_y=-((xyzrObject[1]+_YSizeCam/2)/_YSizeCam*_limAngleCam*2-_limAngleCam);
  // double angle_x=-((xyzrObject[2]+_ZSizeCam/2)/_ZSizeCam*_limAngleCam*2-_limAngleCam);

  double angle_z=(xyzrObject[2])/(_ZSizeCam/2)*_limAngleCamZ;
  double angle_y=(xyzrObject[1])/(_YSizeCam/2)*_limAngleCamY;
  double angle_yr=(xyzrObject[1]+xyzrObject[3])/(_YSizeCam/2)*_limAngleCamY;

  currentVector[0]=xyzrObject[3]/(tan(angle_yr)-tan(angle_y))*cos(angle_z);
  currentVector[1]=(currentVector[0]-_xOffset)*tan(angle_y);
  currentVector[2]=currentVector[0]*tan(angle_z);
  #ifdef DEBUG
  std::cout<<"Vector for Object: \n"<<currentVector<<"\n";
  #endif
  currentVector=_getTransformRotMatrix(alphaDrone, phiDrone).inverse()*currentVector;

  return currentVector;
}

//Get rotation matrix for frame trnsformation
Eigen::Matrix3d DroneController::_getTransformRotMatrix(double alpha, double phi){
  Eigen::Matrix3d RMatrix;
  RMatrix = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(phi,  Eigen::Vector3d::UnitY());
  return RMatrix;
}
