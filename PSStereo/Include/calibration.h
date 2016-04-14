#ifndef CALIB_H
#define CALIB_H

#include "stereo_system.h"
#include "utils.h"
#include <vector>
#include <iterator>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\highgui\highgui.hpp>

using namespace cv;

bool Calibrate(bool getpictures,bool use_pairs_only,Size boardsize, StereoSystem* ssys);
void CalibrateSingleCamera(const vector<string>& imagelist, Size boardSize,Mat& cameraMatrix, Mat& distCoeffs);
void StereoCalib(const vector<string>& imagelist, Size boardSize,Mat calibmatix[], bool useCalibrated, bool showRectified);
bool readStringList( const string& filename, vector<string>& l );

#endif