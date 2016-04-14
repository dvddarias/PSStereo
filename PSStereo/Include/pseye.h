#pragma once

#include <iostream>	// for standard I/O
#include <string> 
#include <sstream>
#include "CLEyeMulticam.h"
#include "utils.h"
#include "stereo_camera.h"
#include <opencv2\core\core.hpp>        

using namespace cv;
using namespace std;

class PSEye: public StereoCamera
{    
private:
	GUID left_cameraGUID;
	GUID right_cameraGUID;
	CLEyeCameraInstance left_cam;
	CLEyeCameraInstance right_cam;	

public:
	PSEye(Resolution resolution, ColorMode mode, float fps, bool automatic_params);
	~PSEye();
	virtual bool Start();
	virtual bool Stop();
	virtual bool SetParameter(CameraParameter param, int value);
	virtual int  GetParameter(CameraParameter param);
	virtual bool GetFrameDimensions(int &width, int &height);
	virtual bool GetStereoPair(Mat& Left, Mat& Right);
	virtual bool GetLeftFrame(Mat& Left);
	virtual bool GetRightFrame(Mat& Right);
	static bool CameraAviable();
	virtual string GetCalibrationPath();
};