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

class Minoru: public StereoCamera
{    
private:
	VideoCapture* left_cam;
	VideoCapture* right_cam;
	Mat* left_gray;
	Mat* right_gray;
	Mat* left_color;
	Mat* right_color;
	Mat* left_color_a;
	Mat* right_color_a;
public:
	Minoru(Resolution resolution, ColorMode mode, float fps, bool automatic_params);
	~Minoru();
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