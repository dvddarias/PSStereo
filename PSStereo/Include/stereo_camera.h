#pragma once

#include <opencv2\core\core.hpp>  
using namespace cv;
using namespace std;

typedef enum
{ 
	MONO=2,
	COLOR=3
}ColorMode;

typedef enum
{ 
	QVGA,
	VGA
}Resolution;

typedef enum
{
	AUTO_GAIN,			
	GAIN,			
	AUTO_EXPOSURE,
	EXPOSURE,		
	AUTO_WHITEBALANCE,
	WHITEBALANCE_RED,		
	WHITEBALANCE_GREEN,	
	WHITEBALANCE_BLUE	
}CameraParameter;

class StereoCamera
{
	public:
		ColorMode _mode;
		Resolution _resolution;
		bool _auto_tune;
		float _fps;

		StereoCamera(Resolution resolution, ColorMode mode, float fps, bool auto_tune){_resolution = resolution; _mode = mode; _fps=fps; _auto_tune = auto_tune;}
		virtual bool Start()=0;
		virtual bool Stop()=0;
		virtual bool SetParameter(CameraParameter param, int value)=0;
		virtual int  GetParameter(CameraParameter param)=0;
		virtual bool GetFrameDimensions(int &width, int &height)=0;
		virtual bool GetStereoPair(Mat& Left, Mat& Right)=0;
		virtual bool GetLeftFrame(Mat& Left)=0;
		virtual bool GetRightFrame(Mat& Right)=0;	
		virtual string GetCalibrationPath()=0;
};