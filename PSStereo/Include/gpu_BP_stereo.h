#ifndef GPU_BP_STEREO_H
#define GPU_BP_STEREO_H

#include "stereo_alg.h"

class BP_gpu:public StereoAlgorithm
{

protected:

	GpuMat gpu_left;
	GpuMat gpu_right;
	GpuMat gpu_aux;
	GpuMat gpu_disp;	
	StereoBeliefPropagation* bp;

public:

	BP_gpu():StereoAlgorithm(){	outputType=CV_16S;	}
	void Compute(Mat left,Mat Right,Mat& disp);
	void Initialize(Size s,string calibration_path);
	void SetParamaters();
	void PrintParamsDescription();
	void BuildDefaultParameters();
	Mat GetUserFirendlyDisparity(Mat disp);
	Mat GetFloatDisparity(Mat disp);
};

#endif

