#pragma once
#define DRoneController

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

class DroneController{
public:
  DroneController(double limAngleCamY, double limAngleCamZ, double Kx, double Ky, double Kz, double YSizeCam, double ZSizeCam);
  std::vector<float> getVectorControl(double heighDrone, std::vector<double> xyzrObject, double alphaDrone, double phiDrone, std::vector<double> desiredVector);
private:
  double _limAngleCamY;
  double _limAngleCamZ;
  double _Kx;
  double _Ky;
  double _Kz;
  double _YSizeCam;
  double _ZSizeCam;
  double _xOffset;
  Eigen::Matrix3d _controlMatrix;
  Eigen::Vector3d _maxControlVector;
  Eigen::Vector3d _getCurrentVector(double heighDrone, Eigen::Vector4d xyzrObject, double alphaDrone, double phiDrone);
  Eigen::Matrix3d _getTransformRotMatrix(double alpha, double phi);
};
