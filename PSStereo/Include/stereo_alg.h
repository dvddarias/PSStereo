#ifndef STEREO_ALG_H
#define STEREO_ALG_H

#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp> 
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\contrib\contrib.hpp>
#include <opencv2\gpu\gpu.hpp>
#include "global_params.h"
#include "utils.h"
#include <iostream>

using namespace cv;
using namespace std;
using namespace cv::gpu;

class StereoAlgorithm
{
protected:
	const char* intrinsic_filename;
    const char* extrinsic_filename;
    const char* disparity_filename;
    const char* point_cloud_filename;
	const char* stereo_pair_filename;
	
	bool _use_calibration;

	float scale;	
    Rect roiL, roiR;

	Mat map11, map12, map21, map22;

public:
	AlgParameters* parameters;
	const char* name;
	bool Update_params;
	int outputType;
	Size img_size;
	Mat Q;
	Mat imgL;
	Mat imgR;
	string* calibration_path;

	StereoAlgorithm::StereoAlgorithm();

	virtual void Compute(Mat left,Mat right,Mat& disp)=0;
	virtual void Initialize(Size s,string calibration_path)=0;
	virtual void SetParamaters()=0;
	virtual void PrintParamsDescription()=0;
	virtual void BuildDefaultParameters()=0;
	virtual Mat GetUserFirendlyDisparity(Mat disp)=0;
	virtual Mat GetFloatDisparity(Mat disp)=0;
	
	void Load(Size s,string calibration_path);
	void ReadCalibration(string calibration_path);
	void ScaleImages();
	void RectifyImages();
	void UpdateParameters();
	void StoreResults(Mat disp, Mat left, Mat right);
	Mat GetPointCloud(Mat disp);
};

void saveXYZ(const char* filename, const Mat& mat);

#endif