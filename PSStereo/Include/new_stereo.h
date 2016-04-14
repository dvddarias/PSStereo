#ifndef NEW_STEREO_H
#define NEW_STEREO_H

#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include "stereo_alg.h"
#include "utils.h"

using namespace cv;
using namespace std;


class NewStereo:public StereoAlgorithm
{
protected:
	int disparities;
	int windows;
	int min_disparity;
	int min_window_size;
	int window_size_step;

	Mat accum;
	Mat ground_truth;

public:
	NewStereo():StereoAlgorithm(){}

	void Compute(Mat left,Mat right,Mat& disp);
	void Initialize(Size s,string calibration_path);
	void SetParamaters();
	void PrintParamsDescription();
	void BuildDefaultParameters();

	Mat GetUserFirendlyDisparity(Mat disp);
	Mat GetFloatDisparity(Mat disp);

	void FillDisparities(Mat& reference, Mat& target, Mat& disp);
	uchar GetDisparity(int row, int col, Mat& reference, Mat& target);
	Mat GetWindow(int row, int col, int size, Mat& image);
	float MatchingCost(Mat base, Mat actual);
	void RowNormalize(Mat& image);
	void ShowAccumulator(int row, int col, Mat& image);
	float SelectDisparity(Mat& accum);
};

#endif