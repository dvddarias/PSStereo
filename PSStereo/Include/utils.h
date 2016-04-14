#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <string>

#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>

using namespace std;
using namespace cv;

struct AlgParameters
{
	float P0;	float P1;	float P2;	float P3;	float P4;
	float P5;	float P6;	float P7;	float P8;	float P9;	
};

string GetExecutablePath();

void ShowImages(Mat& left, Mat& right,const char* label);

int LoadImages(const char* pair,const char* ext,Mat& left,Mat& right);

int LoadImage(const char* pair,const char* name,Mat& img);

void AmplifyColor(Mat& img_disp,Mat& result,int maxdisp);

float ReadDisparity(int row, int col, Mat& img);

double CosAngle(Vec3b v1,Vec3b v2);

double Dist(Vec3b v1,Vec3b v2);

void showHistogram(Mat& img);

Mat GetDFT(Mat& I);

void Wiener2(Mat& src, Mat& dst, int szWindowX = 3, int szWindowY = 3 );

void PrintParameters(AlgParameters* params);

Mat Sharper(Mat& img);

bool CornersVisible(Mat &img, Size boardSize);

float Clamp(float param,float minval,float maxval);

Mat GetGrayRepresentation(Mat disparity);


#endif