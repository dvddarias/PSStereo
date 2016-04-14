#include "pseye.h"	

PSEye::PSEye(Resolution resolution, ColorMode mode, float fps, bool auto_tune):StereoCamera(resolution,mode,fps,auto_tune)
{	
	left_cameraGUID = CLEyeGetCameraUUID(1);
	right_cameraGUID = CLEyeGetCameraUUID(0);
	if(left_cameraGUID.Data1!=815035109)
	{
		left_cameraGUID = CLEyeGetCameraUUID(0);
		right_cameraGUID = CLEyeGetCameraUUID(1);
	}

	left_cam = CLEyeCreateCamera(left_cameraGUID, (CLEyeCameraColorMode)_mode, (CLEyeCameraResolution)_resolution, _fps);  //first the master
	right_cam = CLEyeCreateCamera(right_cameraGUID, (CLEyeCameraColorMode)_mode, (CLEyeCameraResolution)_resolution, _fps);//then the slave

	// Set some camera parameters
	SetParameter( AUTO_GAIN, auto_tune);
	SetParameter( AUTO_EXPOSURE, auto_tune);
	SetParameter( AUTO_WHITEBALANCE, auto_tune);

	if(!auto_tune)
	{
		SetParameter(GAIN, 0);
		SetParameter(EXPOSURE, 511);
		SetParameter(WHITEBALANCE_RED, 125);
		SetParameter(WHITEBALANCE_GREEN, 125);
		SetParameter(WHITEBALANCE_BLUE, 125);
	}
	
}

PSEye::~PSEye()
{
	CLEyeDestroyCamera(left_cam);
	CLEyeDestroyCamera(right_cam);

	left_cam = NULL;
	right_cam = NULL;
}

bool PSEye::Start()
{
	// Start capturing
	return CLEyeCameraStart(left_cam)&&CLEyeCameraStart(right_cam);	
}

bool PSEye::Stop()
{
	// Stop camera capture	
	//first the slave
	//then the master	
	return CLEyeCameraStop(right_cam) && CLEyeCameraStop(left_cam);
}

bool PSEye::SetParameter(CameraParameter param, int value)
{
	CLEyeSetCameraParameter(left_cam, (CLEyeCameraParameter)param, value);
	CLEyeSetCameraParameter(right_cam, (CLEyeCameraParameter)param, value);
	return true;
}

int PSEye::GetParameter(CameraParameter param)
{
	return CLEyeGetCameraParameter(left_cam, (CLEyeCameraParameter)param);
}

bool PSEye::GetFrameDimensions(int &width, int &height)
{
	return CLEyeCameraGetFrameDimensions(left_cam,width,height);
}

bool PSEye::GetStereoPair(Mat& Left, Mat& Right)
{
	uchar* left_data = Left.ptr();
	uchar* right_data= Right.ptr();
	CLEyeCameraGetFrame(left_cam, left_data, 2000);
	CLEyeCameraGetFrame(right_cam, right_data, 0);
	return true;
}

bool PSEye::GetLeftFrame(Mat& Left)
{
	uchar* left_data = Left.ptr();
	CLEyeCameraGetFrame(left_cam, left_data,0);
	return true;
}

bool PSEye::GetRightFrame(Mat& Right)
{
	uchar* right_data= Right.ptr();
	CLEyeCameraGetFrame(right_cam, right_data, 0);
	return true;
}

bool PSEye::CameraAviable()
{
	// Query for number of connected cameras
	if(CLEyeGetCameraCount() != 2)
	{
		printf("2 PS3Eye cameras needed.\n");
		return false;
	}
	return true;
}

string PSEye::GetCalibrationPath()
{
	return string("D:\\Tutorials & Teaching\\!!TESIS\\Projects\\VSStereo\\PSStereo\\Bin\\CalibrationData\\PSEye\\");
}