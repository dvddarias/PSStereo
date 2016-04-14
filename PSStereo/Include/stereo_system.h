#pragma once

#include <iostream>	// for standard I/O
#include <string> 
#include <sstream>
#include "CLEyeMulticam.h"
#include "global_params.h"
#include "utils.h"
#include "noise.h"
#include "stereo_alg.h"
#include "pseye.h"

#include <opencv2\core\core.hpp>        
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace cv;
using namespace std;
using namespace cv::gpu;

#define CAM_LEFT  0
#define CAM_RIGHT  1
#define CAM_PAIR  2

class StereoSystem
{    
private:
	FileStorage pairs;
	FileStorage lefts;
	FileStorage rights;
	float _fps;
	HANDLE _hThread;
	bool _running;
	bool _capturenext;		
	bool _closelist;
	bool _calculateDepth;

	bool _updateAlgParameters;
	AlgParameters* noise_params;

public:
	int pair_cap_count;
	int left_cap_count;
	int right_cap_count;
	int capture_source;
	StereoAlgorithm* alg;
	StereoCamera* cam;

	StereoSystem(StereoCamera* cam, StereoAlgorithm* algorithm = NULL);
	bool StartCaptureLoop();
	void StopCaptureLoop();
	void CloseImageList();
	void CaptureImageFiles(Mat left, Mat right);
	void Run();
	void BuildNiseParams();
	static DWORD WINAPI CaptureThread(LPVOID instance);
	void IncrementParameter(int param);
	void DecrementParameter(int param);	
	void CaptureNextFrame();
};

void MenuInteraction(StereoSystem* ssys);

