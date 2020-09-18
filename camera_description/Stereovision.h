#ifndef Stereovision_h
#define Stereovision_h

#include "CameraCalibration.h"


class Stereovision(){
	CameraCalibration objR, objT;
	Mat K1, K2;
	Mat distCoeffsR, distCoeffsT;
	Mat H, F, E;
    Mat R, t;
public:
	void findExtrinsics();
}
#endif 
