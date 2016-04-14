#ifndef CPU_SGBM_STEREO_H
#define CPU_SGBM_STEREO_H

#include "stereo_alg.h" 

class SGBM_cpu: public StereoAlgorithm
{
protected:

    StereoSGBM sgbm;


public:

	SGBM_cpu():StereoAlgorithm(){	outputType=CV_16S;	}
	void Compute(Mat left,Mat Right,Mat& disp);
	void Initialize(Size s, string calibration_path);
	void SetParamaters();
	void PrintParamsDescription();
	void BuildDefaultParameters();
	Mat GetUserFirendlyDisparity(Mat disp);
	Mat GetFloatDisparity(Mat disp);
};

#endif