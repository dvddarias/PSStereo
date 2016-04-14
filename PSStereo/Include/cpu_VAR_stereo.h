#ifndef CPU_VAR_STEREO_H
#define CPU_VAR_STEREO_H

#include "stereo_alg.h" 

class VAR_cpu: public StereoAlgorithm
{
protected:

	StereoVar var;


public:

	VAR_cpu():StereoAlgorithm(){	outputType=CV_8U;	}
	void Compute(Mat left,Mat Right,Mat& disp);
	void Initialize(Size s, string calibration_path);
	void SetParamaters();
	void PrintParamsDescription();
	void BuildDefaultParameters();
	Mat GetUserFirendlyDisparity(Mat disp);
	Mat GetFloatDisparity(Mat disp);
};

#endif