#include "minoru.h"	
//falta tener en cuenta los parámetros
Minoru::Minoru(Resolution resolution, ColorMode mode, float fps, bool auto_tune):StereoCamera(resolution,mode,fps,auto_tune)
{	
	left_cam = new VideoCapture();
	right_cam = new VideoCapture();
	left_color = new Mat(Size(640,480),CV_8UC3);
	right_color = new Mat(Size(640,480),CV_8UC3);
	if(_mode==MONO)
	{
		left_gray = new Mat(Size(640,480),CV_8U);
		right_gray = new Mat(Size(640,480),CV_8U);
	}
	else
	{
		left_color_a = new Mat(Size(640,480),CV_8UC4);
		right_color_a = new Mat(Size(640,480),CV_8UC4);
	}
}

Minoru::~Minoru()
{
	left_cam->release();
	right_cam->release();
}

bool Minoru::Start()
{
	return left_cam->open(0)&&right_cam->open(1);
}

bool Minoru::Stop()
{
	left_cam->release();
	right_cam->release();
	return true;
}
//falta
bool Minoru::SetParameter(CameraParameter param, int value)
{
	return false;
}
//falta
int Minoru::GetParameter(CameraParameter param)
{
	return false;
}

bool Minoru::GetFrameDimensions(int &width, int &height)
{
	width = _resolution==QVGA?320:640;
	height = _resolution==QVGA?240:480;
	return true;
}

bool Minoru::GetStereoPair(Mat& Left, Mat& Right)
{
	(*left_cam).grab();
	(*right_cam).grab();
	bool l = (*left_cam).retrieve(*left_color);
	bool r = (*right_cam).retrieve(*right_color);	
	if(_mode==MONO)
	{
		cvtColor(*left_color,*left_gray,CV_BGR2GRAY);
		cvtColor(*right_color,*right_gray,CV_BGR2GRAY);
		if(_resolution==QVGA)
		{
			resize(*left_gray,Left,Size(320,240));
			resize(*right_gray,Right,Size(320,240));
		}
		else
		{
			left_gray->copyTo(Left);
			right_gray->copyTo(Right);
		}
	}
	else
	{
		cvtColor(*left_color,*left_color_a,CV_BGR2BGRA);
		cvtColor(*right_color,*right_color_a,CV_BGR2BGRA);
		if(_resolution==QVGA)
		{
			resize(*left_color_a,Left,Size(320,240));
			resize(*right_color_a,Right,Size(320,240));
		}
		else
		{
			left_color_a->copyTo(Left);
			right_color_a->copyTo(Right);
		}
	}
	return l&&r;
}

bool Minoru::GetLeftFrame(Mat& Left)
{
	(*left_cam).grab();
	bool l = (*left_cam).retrieve(*left_color);
	if(_mode==MONO)
	{
		cvtColor(*left_color,*left_gray,CV_BGR2GRAY);
		if(_resolution==QVGA) resize(*left_gray,Left,Size(320,240));
		else left_gray->copyTo(Left);
	}
	else
	{
		cvtColor(*left_color,*left_color_a,CV_BGR2BGRA);
		if(_resolution==QVGA) resize(*left_color_a,Left,Size(320,240));
		else left_color_a->copyTo(Left);
	}
	return l;
}

bool Minoru::GetRightFrame(Mat& Right)
{
	(*right_cam).grab();
	bool r = (*right_cam).retrieve(*right_color);	
	if(_mode==MONO)
	{
		cvtColor(*right_color,*right_gray,CV_BGR2GRAY);
		if(_resolution==QVGA) resize(*right_gray,Right,Size(320,240));
		else right_gray->copyTo(Right);
	}
	else
	{
		cvtColor(*right_color,*right_color_a,CV_BGR2BGRA);
		if(_resolution==QVGA) resize(*right_color_a,Right,Size(320,240));
		else right_color_a->copyTo(Right);
	}
	return r;
}

bool Minoru::CameraAviable()
{	
	return true;
}

string Minoru::GetCalibrationPath()
{
	return string("D:\\Tutorials & Teaching\\!!TESIS\\Projects\\VSStereo\\PSStereo\\Bin\\CalibrationData\\Minoru\\");
}