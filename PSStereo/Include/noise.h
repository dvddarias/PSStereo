#ifndef NOISE_WORK_H
#define NOISE_WORK_H

#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <string>
#include "utils.h"

#include <opencv2\gpu\gpu.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace std;
using namespace cv;

class DeNoiser
{
	Mat** images;
	Mat Accum;
	int image_count;
	int w;
	int h;
	int hist_size;
	bool first_time;
public:
	DeNoiser::DeNoiser(int width,int height,int history_size);
	void DeNoiseStatic(Mat &image, AlgParameters* params);
	void DeNoiseSemiStatic(Mat &image, AlgParameters* params);
};

class Noiser
{
	Mat* images;
	Mat cleanImage;
	int w;
	int h;
	int actual;
	int total;
	int skip_amount;
	bool ready;
public:
	Noiser::Noiser(int width,int height,int testsize,int skip_amount);
	void Accumulate(Mat img);
	void ComputeCleanImage();
	void EstimateNoise();
	void Reset();
};

class DepthMapJudge
{
	Vec4f accumulated;
	int actual;
	int total;
	float radius;
	float maxZ,minZ;
	//To store neighbors and calculate standard deviation 
	Mat points;
	int ws;
	
	Vec2f CalculateNeighbours(Vec3f center, int x, int y, Mat xyz);
public:
	DepthMapJudge::DepthMapJudge(int total, float radius, float minZ,float maxZ);
	void Judge(Mat xyz);
	Vec4f Measure(Mat xyz);
};

#endif