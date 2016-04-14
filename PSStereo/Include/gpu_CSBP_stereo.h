#ifndef GPU_CSBP_STEREO_H
#define GPU_CSBP_STEREO_H

#include "stereo_alg.h"

class CSBP_gpu:public StereoAlgorithm
{

protected:

	GpuMat gpu_left;
	GpuMat gpu_right;
	GpuMat gpu_refined;
	GpuMat gpu_aux;
	GpuMat gpu_disp;	
	StereoConstantSpaceBP* csbp;
	DisparityBilateralFilter* dbf;

public:

	CSBP_gpu():StereoAlgorithm(){	outputType=CV_16S;	}
	void Compute(Mat left,Mat Right,Mat& disp);
	void Initialize(Size s,string calibration_path);
	void SetParamaters();
	void PrintParamsDescription();
	void BuildDefaultParameters();
	Mat GetUserFirendlyDisparity(Mat disp);
	Mat GetFloatDisparity(Mat disp);
};

#endif

